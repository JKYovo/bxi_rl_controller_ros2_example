"""Explicit-provider ONNX worker isolated from the BXI controller process."""

from __future__ import annotations

import argparse
import ctypes
from multiprocessing.connection import Connection
import os
from pathlib import Path
import time

import numpy as np
import onnxruntime as ort

OBS_DIM = 604
ACTION_DIM = 29
KV_SHAPE = (1, 2, 1, 32, 4, 64)
PROVIDER_ENV = "HOLOMOTION_ONNX_PROVIDER"
SUPPORTED_PROVIDERS = ("cuda", "cpu")


def require_cuda_runtime() -> None:
    """Require the deployment CUDA runtime without accepting CPU fallback."""

    preload = getattr(ort, "preload_dlls", None)
    if callable(preload):
        preload()
    providers = tuple(ort.get_available_providers())
    if "CUDAExecutionProvider" not in providers:
        raise RuntimeError(
            "HoloMotion requires CUDAExecutionProvider; available providers are "
            f"{providers}"
        )
    try:
        ctypes.CDLL("libcublasLt.so.12")
    except OSError as exc:
        raise RuntimeError(
            "HoloMotion requires libcuBLAS Lt 12 (libcublasLt.so.12); "
            "CUDA is incomplete and CPU fallback is disabled"
        ) from exc


def require_cpu_runtime() -> None:
    """Require the CPU execution provider without selecting it as fallback."""

    providers = tuple(ort.get_available_providers())
    if "CPUExecutionProvider" not in providers:
        raise RuntimeError(
            "HoloMotion requires CPUExecutionProvider; available providers are "
            f"{providers}"
        )


def _validate_contract(session: ort.InferenceSession) -> dict[str, str]:
    inputs = {node.name: node for node in session.get_inputs()}
    outputs = {node.name: node for node in session.get_outputs()}
    expected_inputs = {
        "obs": [1, OBS_DIM],
        "past_key_values": list(KV_SHAPE),
        "step_idx": [1],
    }
    for name, shape in expected_inputs.items():
        if name not in inputs or list(inputs[name].shape) != shape:
            raise ValueError(
                f"unexpected HoloMotion input {name}: "
                f"{getattr(inputs.get(name), 'shape', None)}"
            )
    if "actions" not in outputs or list(outputs["actions"].shape) != [1, ACTION_DIM]:
        raise ValueError("HoloMotion actions output is not [1,29]")
    if (
        "present_key_values" not in outputs
        or list(outputs["present_key_values"].shape) != list(KV_SHAPE)
    ):
        raise ValueError("HoloMotion KV-cache output shape changed")
    return dict(session.get_modelmeta().custom_metadata_map)


def _create_session(
    model_path: Path, provider: str = "cuda"
) -> ort.InferenceSession:
    if not model_path.is_file():
        raise FileNotFoundError(f"HoloMotion model does not exist: {model_path}")
    provider = provider.strip().lower()
    if provider not in SUPPORTED_PROVIDERS:
        raise ValueError(
            f"unsupported HoloMotion ONNX provider {provider!r}; "
            f"expected one of {SUPPORTED_PROVIDERS}"
        )
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if provider == "cuda":
        require_cuda_runtime()
        options.add_session_config_entry("session.disable_cpu_ep_fallback", "1")
        configured_providers = [("CUDAExecutionProvider", {"device_id": 0})]
        expected_provider = "CUDAExecutionProvider"
    else:
        require_cpu_runtime()
        configured_providers = ["CPUExecutionProvider"]
        expected_provider = "CPUExecutionProvider"
    session = ort.InferenceSession(
        str(model_path),
        sess_options=options,
        providers=configured_providers,
    )
    providers = tuple(session.get_providers())
    if not providers or providers[0] != expected_provider:
        raise RuntimeError(
            f"HoloMotion requires an active {expected_provider} for the 50 Hz "
            f"BXI runtime; actual providers are {providers}"
        )
    return session


def _infer(
    session: ort.InferenceSession,
    observation: object,
    kv_cache: object,
    step_index: object,
) -> tuple[np.ndarray, np.ndarray]:
    obs = np.asarray(observation, dtype=np.float32)
    kv = np.asarray(kv_cache, dtype=np.float32)
    step = np.asarray([int(step_index)], dtype=np.int64)
    if obs.shape != (1, OBS_DIM) or not np.all(np.isfinite(obs)):
        raise ValueError(f"invalid HoloMotion worker observation: {obs.shape}")
    if kv.shape != KV_SHAPE or not np.all(np.isfinite(kv)):
        raise ValueError(f"invalid HoloMotion worker KV cache: {kv.shape}")
    actions, present = session.run(
        ["actions", "present_key_values"],
        {"obs": obs, "past_key_values": kv, "step_idx": step},
    )
    actions = np.asarray(actions, dtype=np.float32)
    present = np.asarray(present, dtype=np.float32)
    if actions.shape != (1, ACTION_DIM) or not np.all(np.isfinite(actions)):
        raise ValueError("HoloMotion worker returned an invalid action")
    if present.shape != KV_SHAPE or not np.all(np.isfinite(present)):
        raise ValueError("HoloMotion worker returned an invalid KV cache")
    return actions, present


def serve(
    connection: Connection, model_path: Path, provider: str = "cuda"
) -> None:
    try:
        session = _create_session(model_path, provider)
        metadata = _validate_contract(session)
        started_at = time.perf_counter()
        _infer(
            session,
            np.zeros((1, OBS_DIM), dtype=np.float32),
            np.zeros(KV_SHAPE, dtype=np.float32),
            0,
        )
        cold_warmup_ms = (time.perf_counter() - started_at) * 1000.0
        connection.send(
            {
                "ok": True,
                "providers": tuple(session.get_providers()),
                "metadata": metadata,
                "cold_warmup_ms": cold_warmup_ms,
            }
        )
    except BaseException as exc:
        try:
            connection.send({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        finally:
            connection.close()
        return

    try:
        while True:
            try:
                request = connection.recv()
            except EOFError:
                return
            if not isinstance(request, dict):
                connection.send({"ok": False, "error": "invalid worker request"})
                continue
            operation = request.get("op")
            if operation == "close":
                return
            if operation != "infer":
                connection.send(
                    {"ok": False, "error": f"unsupported worker operation: {operation}"}
                )
                continue
            try:
                actions, present = _infer(
                    session,
                    request.get("obs"),
                    request.get("past_key_values"),
                    request.get("step_idx"),
                )
                connection.send(
                    {
                        "ok": True,
                        "actions": actions,
                        "present_key_values": present,
                    }
                )
            except BaseException as exc:
                connection.send(
                    {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
                )
    finally:
        connection.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection-fd", type=int, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument(
        "--provider",
        choices=SUPPORTED_PROVIDERS,
        default=os.environ.get(PROVIDER_ENV, "cuda").strip().lower(),
    )
    args = parser.parse_args()
    serve(Connection(args.connection_fd), args.model.resolve(), args.provider)


if __name__ == "__main__":
    main()
