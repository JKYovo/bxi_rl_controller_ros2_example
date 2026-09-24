"""BXI Mod adapter for the 2217-D external-reference ELF3 RGMT actor.

The RGMT ABI and quaternion math are adapted from MelodyAI/bxi_elf3_ws
``models/rgmt2.py`` at commit 6ef008f343eb6acb8d4d40aeebd14df1468e7eec.
This adapter replaces the legacy controller integration with named BXI joint
contracts and the existing ordered HoloRetarget stream.
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from bxi_example_py_elf3.framework.inference import (
    InferenceFrame,
    JointPolicy,
    ModelSpec,
    PolicyJointContract,
    PolicyOutput,
)
from bxi_example_py_elf3.framework.inference.runtime import (
    InferenceRuntime,
    default_runtime,
)
from bxi_example_py_elf3.policies import (
    ELF3_ISAAC_JOINTS,
    ELF3_ISAAC_PARAMETERS,
    ELF3_POLICY_JOINTS,
)

from .rgmt_reference import (
    CURRENT_FRAME_OFFSET,
    WINDOW_SIZE,
    ReferenceReceiver,
    ReferenceWindow,
)

ACTION_DIM = 29
OBS_DIM = 2217
COMMAND_TOKEN_DIM = 61
PROPRIO_TOKEN_DIM = 93
PROPRIO_HISTORY_LENGTH = 10
CONTROL_FPS = 50.0
CONTROL_DT = 1.0 / CONTROL_FPS
COMMAND_TOKEN_LAYOUT = (
    "reference_root_ang_vel_b[3],reference_joint_pos[29],"
    "reference_joint_vel[29]"
)
OBSERVATION_LAYOUT = (
    "rgmt_command[21x61],motion_anchor_ori_b[6],rgmt_proprio[10x93]"
)

REFERENCE_JOINT_NAMES = ELF3_POLICY_JOINTS.names
REFERENCE_TO_ISAAC = np.asarray(
    tuple(
        REFERENCE_JOINT_NAMES.index(name) for name in ELF3_ISAAC_JOINTS.names
    ),
    dtype=np.int64,
)


class LoggerLike(Protocol):
    def info(self, message: str) -> None: ...
    def warning(self, message: str) -> None: ...
    def error(self, message: str) -> None: ...


def _metadata_floats(
    metadata: dict[str, str], key: str, length: int
) -> np.ndarray:
    if key not in metadata:
        raise ValueError(f"RGMT ONNX metadata is missing {key}")
    try:
        values = np.asarray(
            [
                float(value)
                for value in metadata[key].split(",")
                if value.strip()
            ],
            dtype=np.float32,
        )
    except ValueError as exc:
        raise ValueError(f"RGMT ONNX metadata {key} is not numeric") from exc
    if values.shape != (length,) or not np.all(np.isfinite(values)):
        raise ValueError(
            f"RGMT ONNX metadata {key} must contain {length} finite values"
        )
    return values


def _metadata_ints(metadata: dict[str, str], key: str) -> np.ndarray:
    if key not in metadata:
        raise ValueError(f"RGMT ONNX metadata is missing {key}")
    try:
        return np.asarray(
            [
                int(float(value))
                for value in metadata[key].split(",")
                if value.strip()
            ],
            dtype=np.int64,
        )
    except ValueError as exc:
        raise ValueError(f"RGMT ONNX metadata {key} is not integral") from exc


def _normalize_quat(value: object) -> np.ndarray:
    quaternion = np.asarray(value, dtype=np.float32)
    if quaternion.shape[-1:] != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError(f"invalid quaternion array: {quaternion.shape}")
    norm = np.linalg.norm(quaternion, axis=-1, keepdims=True)
    if np.any(norm <= np.finfo(np.float32).eps):
        raise ValueError("quaternion contains a zero-length value")
    return (quaternion / norm).astype(np.float32, copy=False)


def quat_conjugate_wxyz(quaternion: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternion, dtype=np.float32).copy()
    result[..., 1:] *= -1.0
    return result


def quat_multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    left = np.asarray(left, dtype=np.float32)
    right = np.asarray(right, dtype=np.float32)
    w1, x1, y1, z1 = np.moveaxis(left, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(right, -1, 0)
    return np.stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ),
        axis=-1,
    ).astype(np.float32, copy=False)


def quat_rotate_inverse_wxyz(
    quaternion: np.ndarray, vector: np.ndarray
) -> np.ndarray:
    quaternion = _normalize_quat(quaternion)
    vector = np.asarray(vector, dtype=np.float32)
    if quaternion.shape[:-1] != vector.shape[:-1] or vector.shape[-1:] != (3,):
        raise ValueError(
            "quaternion/vector shapes must be [...,4] and [...,3] with "
            "identical leading dimensions, got "
            f"{quaternion.shape} and {vector.shape}"
        )
    scalar = quaternion[..., :1]
    xyz = quaternion[..., 1:]
    cross = 2.0 * np.cross(xyz, vector)
    return (vector - scalar * cross + np.cross(xyz, cross)).astype(
        np.float32, copy=False
    )


def yaw_quaternion_wxyz(quaternion: np.ndarray) -> np.ndarray:
    quaternion = _normalize_quat(quaternion)
    w, x, y, z = np.moveaxis(quaternion, -1, 0)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    zeros = np.zeros_like(yaw)
    return np.stack(
        (np.cos(0.5 * yaw), zeros, zeros, np.sin(0.5 * yaw)), axis=-1
    ).astype(np.float32, copy=False)


def quaternion_to_rotation_matrix_wxyz(quaternion: np.ndarray) -> np.ndarray:
    q = _normalize_quat(quaternion)
    w, x, y, z = np.moveaxis(q, -1, 0)
    return (
        np.stack(
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
            axis=-1,
        )
        .reshape(q.shape[:-1] + (3, 3))
        .astype(np.float32, copy=False)
    )


def _time_gradient(values: np.ndarray) -> np.ndarray:
    data = np.asarray(values, dtype=np.float32)
    if data.shape[0] < 2:
        raise ValueError("time gradient requires at least two frames")
    result = np.empty_like(data)
    result[0] = (data[1] - data[0]) * CONTROL_FPS
    result[-1] = (data[-1] - data[-2]) * CONTROL_FPS
    result[1:-1] = (data[2:] - data[:-2]) * (0.5 * CONTROL_FPS)
    return result


def _quaternion_angular_velocity_world(quaternions: np.ndarray) -> np.ndarray:
    quaternion = _normalize_quat(quaternions)
    if quaternion.shape[0] < 2:
        raise ValueError("angular velocity requires at least two quaternions")
    low = np.concatenate(
        (quaternion[:1], quaternion[:-2], quaternion[-2:-1]), axis=0
    )
    high = np.concatenate(
        (quaternion[1:2], quaternion[2:], quaternion[-1:]), axis=0
    )
    relative = _normalize_quat(
        quat_multiply_wxyz(high, quat_conjugate_wxyz(low))
    )
    relative[relative[:, 0] < 0.0] *= -1.0
    xyz = relative[:, 1:]
    magnitude = np.linalg.norm(xyz, axis=1)
    angle = 2.0 * np.arctan2(magnitude, relative[:, 0])
    scale = np.empty_like(angle)
    small = np.abs(angle) <= 1.0e-6
    scale[small] = 2.0
    scale[~small] = angle[~small] / magnitude[~small]
    inverse_dt = np.full(
        quaternion.shape[0], 0.5 * CONTROL_FPS, dtype=np.float32
    )
    inverse_dt[[0, -1]] = CONTROL_FPS
    return (xyz * scale[:, None] * inverse_dt[:, None]).astype(np.float32)


@dataclass(frozen=True, slots=True)
class RgmtReferenceFeatures:
    joint_position_isaac: np.ndarray
    joint_velocity_isaac: np.ndarray
    torso_quat_wxyz: np.ndarray
    torso_angular_velocity_world: np.ndarray


def reference_features_from_qpos(qpos: np.ndarray) -> RgmtReferenceFeatures:
    data = np.asarray(qpos, dtype=np.float32)
    if data.shape != (WINDOW_SIZE, 36):
        raise ValueError(
            f"RGMT reference window must have shape ({WINDOW_SIZE}, 36), "
            f"got {data.shape}"
        )
    if not np.all(np.isfinite(data)):
        raise ValueError("RGMT reference window contains NaN or Inf")
    joint_position = data[:, 7:][:, REFERENCE_TO_ISAAC]
    torso_quat = _normalize_quat(data[:, 3:7])
    return RgmtReferenceFeatures(
        joint_position_isaac=joint_position.astype(np.float32, copy=False),
        joint_velocity_isaac=_time_gradient(joint_position),
        torso_quat_wxyz=torso_quat,
        torso_angular_velocity_world=_quaternion_angular_velocity_world(
            torso_quat
        ),
    )


class RgmtObservationBuilder:
    def __init__(
        self,
        default_joint_position: np.ndarray,
        *,
        reference_yaw_mode: str,
    ) -> None:
        if reference_yaw_mode not in {"none", "initial", "continuous"}:
            raise ValueError(
                "reference_yaw_mode must be none, initial, or continuous"
            )
        self.default_joint_position = np.asarray(
            default_joint_position, dtype=np.float32
        ).reshape(ACTION_DIM)
        self.reference_yaw_mode = reference_yaw_mode
        self.obs = np.zeros((1, OBS_DIM), dtype=np.float32)
        self._history: np.ndarray | None = None
        self._reference_yaw_delta: np.ndarray | None = None

    def reset(self) -> None:
        self.obs.fill(0.0)
        self._history = None
        self._reference_yaw_delta = None

    def _aligned_reference_quaternion(
        self, robot_quat: np.ndarray, reference_quat: np.ndarray
    ) -> np.ndarray:
        if self.reference_yaw_mode == "none":
            return reference_quat
        if (
            self.reference_yaw_mode == "continuous"
            or self._reference_yaw_delta is None
        ):
            self._reference_yaw_delta = quat_multiply_wxyz(
                yaw_quaternion_wxyz(robot_quat),
                quat_conjugate_wxyz(yaw_quaternion_wxyz(reference_quat)),
            )
        return _normalize_quat(
            quat_multiply_wxyz(self._reference_yaw_delta, reference_quat)
        )

    def _anchor_orientation_6d(
        self, robot_quat: np.ndarray, reference_quat: np.ndarray
    ) -> np.ndarray:
        aligned = self._aligned_reference_quaternion(
            robot_quat, reference_quat
        )
        relative = _normalize_quat(
            quat_multiply_wxyz(quat_conjugate_wxyz(robot_quat), aligned)
        )
        return quaternion_to_rotation_matrix_wxyz(relative)[:, :2].reshape(-1)

    def build(
        self,
        *,
        features: RgmtReferenceFeatures,
        robot_joint_position: np.ndarray,
        robot_joint_velocity: np.ndarray,
        robot_quat_wxyz: np.ndarray,
        robot_angular_velocity_body: np.ndarray,
        last_residual_action: np.ndarray,
    ) -> np.ndarray:
        robot_quat = _normalize_quat(
            np.asarray(robot_quat_wxyz, dtype=np.float32).reshape(4)
        )
        robot_omega = np.asarray(
            robot_angular_velocity_body, dtype=np.float32
        ).reshape(3)
        if not np.all(np.isfinite(robot_omega)):
            raise ValueError("robot angular velocity contains NaN or Inf")
        reference_ang_vel_body = quat_rotate_inverse_wxyz(
            features.torso_quat_wxyz,
            features.torso_angular_velocity_world,
        )
        command = np.concatenate(
            (
                reference_ang_vel_body,
                features.joint_position_isaac,
                features.joint_velocity_isaac,
            ),
            axis=1,
        ).astype(np.float32, copy=False)
        if command.shape != (WINDOW_SIZE, COMMAND_TOKEN_DIM):
            raise RuntimeError(f"invalid RGMT command shape: {command.shape}")
        anchor_orientation = self._anchor_orientation_6d(
            robot_quat,
            features.torso_quat_wxyz[CURRENT_FRAME_OFFSET],
        )
        projected_gravity = quat_rotate_inverse_wxyz(
            robot_quat,
            np.asarray((0.0, 0.0, -1.0), dtype=np.float32),
        )
        proprio = np.concatenate(
            (
                projected_gravity,
                robot_omega,
                np.asarray(robot_joint_position, dtype=np.float32)
                - self.default_joint_position,
                np.asarray(robot_joint_velocity, dtype=np.float32),
                np.asarray(last_residual_action, dtype=np.float32),
            )
        ).astype(np.float32, copy=False)
        if proprio.shape != (PROPRIO_TOKEN_DIM,):
            raise RuntimeError(f"invalid RGMT proprio shape: {proprio.shape}")
        if self._history is None:
            self._history = np.repeat(
                proprio[None, :], PROPRIO_HISTORY_LENGTH, axis=0
            )
        else:
            self._history[:-1] = self._history[1:]
            self._history[-1] = proprio
        self.obs[0] = np.concatenate(
            (
                command.reshape(-1),
                anchor_orientation,
                self._history.reshape(-1),
            )
        )
        if not np.all(np.isfinite(self.obs)):
            raise ValueError("RGMT observation contains NaN or Inf")
        return self.obs


class RgmtPolicy(JointPolicy):
    """Run the external-reference RGMT actor inside the BXI scheduler."""

    joint_contract = PolicyJointContract(
        observation=ELF3_ISAAC_JOINTS,
        action=ELF3_ISAAC_JOINTS,
    )

    def __init__(
        self,
        *,
        model_path: Path,
        receiver: ReferenceReceiver,
        reference_yaw_mode: str = "initial",
        runtime: InferenceRuntime | None = None,
    ) -> None:
        super().__init__()
        self.model_path = Path(model_path).resolve()
        self.receiver = receiver
        selected_runtime = runtime or default_runtime()
        spec = ModelSpec.onnx(
            self.model_path,
            input_names=("obs",),
            output_names=("actions",),
            providers=("CPUExecutionProvider",),
        )
        self._backend = selected_runtime.open_backend(
            spec, backend="onnxruntime"
        )
        try:
            metadata = dict(self._backend.metadata)
            self._validate_contract(metadata)
            self.default_joint_position = _metadata_floats(
                metadata,
                "policy_default_joint_pos",
                ACTION_DIM,
            )
            self.action_scale = _metadata_floats(
                metadata, "policy_action_scale", ACTION_DIM
            )
            self.calibration_offset = _metadata_floats(
                metadata,
                "reference_action_calibration_offset",
                ACTION_DIM,
            )
            self.actual_default_joint_position = (
                self.default_joint_position + self.calibration_offset
            ).astype(np.float32, copy=False)
            clip_text = metadata["policy_action_clip"]
            self.action_clip = (
                None if clip_text == "none" else float(clip_text)
            )
            if self.action_clip is not None and (
                not math.isfinite(self.action_clip) or self.action_clip <= 0.0
            ):
                raise ValueError("policy_action_clip must be positive or none")
            self.requires_recovery_latch = (
                metadata.get("deployment_requires_recovery_latch") == "True"
            )
            if self._backend.input_shape("obs") != (1, OBS_DIM):
                raise ValueError(
                    f"RGMT obs input must be (1,{OBS_DIM}), got "
                    f"{self._backend.input_shape('obs')}"
                )
            if self._backend.output_shape("actions") != (1, ACTION_DIM):
                raise ValueError(
                    f"RGMT actions output must be (1,{ACTION_DIM}), got "
                    f"{self._backend.output_shape('actions')}"
                )
        except BaseException:
            self._backend.close()
            raise

        self.kp = ELF3_ISAAC_PARAMETERS.kp
        self.kd = ELF3_ISAAC_PARAMETERS.kd
        self.observation = RgmtObservationBuilder(
            self.actual_default_joint_position,
            reference_yaw_mode=reference_yaw_mode,
        )
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._target = np.empty(ACTION_DIM, dtype=np.float32)
        self._inputs = {"obs": self.observation.obs}
        warmup_started = time.perf_counter()
        self._backend.warmup(self._inputs, 1)
        self.cold_warmup_ms = (time.perf_counter() - warmup_started) * 1000.0
        self.providers = ("CPUExecutionProvider",)
        self.last_inference_ms = 0.0
        self.last_reference_frame = -1
        self.started = False
        self.should_exit_to_normal = False
        self.last_status = "waiting_reference"
        self._stream_epoch: int | None = None
        self._logger: LoggerLike | None = None
        self._last_reported_status: str | None = None
        self.publish_output(self.default_joint_position, self.kp, self.kd)

    @staticmethod
    def _validate_contract(metadata: dict[str, str]) -> None:
        required_scalars = {
            "export_format": "external_reference_actor",
            "export_format_version": "3",
            "quaternion_convention": "wxyz",
            "history_order": "oldest_to_newest",
            "action_semantics": "reference_joint_position_residual",
            "policy_observation_dim": str(OBS_DIM),
            "policy_action_dim": str(ACTION_DIM),
            "rgmt_command_window_size": str(WINDOW_SIZE),
            "rgmt_command_token_dim": str(COMMAND_TOKEN_DIM),
            "rgmt_command_token_layout": COMMAND_TOKEN_LAYOUT,
            "policy_observation_layout": OBSERVATION_LAYOUT,
            "observation_names": (
                "rgmt_command,motion_anchor_ori_b,rgmt_proprio"
            ),
            "anchor_body_name": "torso_link",
        }
        for key, expected in required_scalars.items():
            actual = metadata.get(key)
            if actual != expected:
                raise ValueError(
                    f"RGMT ONNX metadata {key}={actual!r}, "
                    f"expected {expected!r}"
                )
        if tuple(metadata.get("joint_names", "").split(",")) != (
            ELF3_ISAAC_JOINTS.names
        ):
            raise ValueError(
                "RGMT ONNX joint_names do not match ELF3 Isaac order"
            )
        if not np.array_equal(
            _metadata_ints(metadata, "observation_history_lengths"),
            np.asarray((1, 1, PROPRIO_HISTORY_LENGTH), dtype=np.int64),
        ):
            raise ValueError("RGMT observation history lengths changed")
        if not np.array_equal(
            _metadata_ints(metadata, "command_window_offsets"),
            np.arange(-10, 11, dtype=np.int64),
        ):
            raise ValueError("RGMT command window offsets changed")
        if not math.isclose(
            float(metadata.get("policy_fps", "nan")),
            CONTROL_FPS,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ) or not math.isclose(
            float(metadata.get("policy_control_dt", "nan")),
            CONTROL_DT,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            raise ValueError("RGMT policy rate is not 50 Hz")

    def bind_logger(self, logger: LoggerLike) -> None:
        self._logger = logger

    @property
    def stream_epoch(self) -> int | None:
        return self._stream_epoch

    def _set_status(self, status: str) -> None:
        self.last_status = status
        if status == self._last_reported_status or self._logger is None:
            return
        self._last_reported_status = status
        if status == "reference_stale":
            self._logger.warning(f"RGMT status: {status}")
        else:
            self._logger.info(f"RGMT status: {status}")

    def _reset_recurrent_state(self) -> None:
        self.last_action.fill(0.0)
        self.observation.reset()
        self.started = False

    def reset(self, frame: InferenceFrame | None = None) -> None:
        if frame is not None:
            self.bind_joints(frame)
        self._reset_recurrent_state()
        self.should_exit_to_normal = False
        self.last_reference_frame = -1
        self._stream_epoch = None
        self.receiver.reset_playout_to_latest()
        self.publish_output(self.default_joint_position, self.kp, self.kd)
        self._set_status("waiting_reference")

    def _reference_or_wait(self) -> ReferenceWindow | None:
        window = self.receiver.peek_window()
        if (
            window is not None
            and window.receive_age_s <= self.receiver.max_source_age_s
        ):
            return window
        if (
            self.started
            and self.receiver.latest_receive_age()
            > self.receiver.max_source_age_s
        ):
            self.should_exit_to_normal = True
            self._set_status("reference_stale")
        else:
            self._set_status("waiting_reference")
        return None

    def step(
        self,
        frame: InferenceFrame,
        dt: float,
        *,
        advance: bool = True,
    ) -> PolicyOutput:
        del dt
        if not advance:
            return self.output
        window = self._reference_or_wait()
        if window is None:
            return self.output
        if (
            self._stream_epoch is not None
            and window.stream_epoch != self._stream_epoch
        ):
            self._reset_recurrent_state()
        self._stream_epoch = window.stream_epoch
        joints = self.bind_joints(frame)
        features = reference_features_from_qpos(window.qpos)
        self.observation.build(
            features=features,
            robot_joint_position=joints.position,
            robot_joint_velocity=joints.velocity,
            robot_quat_wxyz=frame.quat_wxyz,
            robot_angular_velocity_body=frame.angular_velocity,
            last_residual_action=self.last_action,
        )
        started_at = time.perf_counter()
        outputs = self._backend.run(self._inputs)
        self.last_inference_ms = (time.perf_counter() - started_at) * 1000.0
        action = np.asarray(outputs["actions"], dtype=np.float32).reshape(
            ACTION_DIM
        )
        if not np.all(np.isfinite(action)):
            raise ValueError("RGMT action contains NaN or Inf")
        if self.action_clip is not None:
            np.clip(action, -self.action_clip, self.action_clip, out=action)
        self.last_action[:] = action
        np.multiply(action, self.action_scale, out=self._target)
        self._target += self.calibration_offset
        self._target += features.joint_position_isaac[CURRENT_FRAME_OFFSET]
        if not np.all(np.isfinite(self._target)):
            raise ValueError("RGMT target contains NaN or Inf")
        self.receiver.advance_after_successful_step(window.frame_index)
        self.last_reference_frame = window.frame_index
        self.started = True
        self._set_status("tracking")
        return self.publish_output(self._target, self.kp, self.kd)

    def close(self) -> None:
        self._backend.close()

    def decode_into(self, outputs) -> None:
        raise NotImplementedError("RgmtPolicy uses its ordered reference step")


__all__ = [
    "ACTION_DIM",
    "COMMAND_TOKEN_DIM",
    "OBS_DIM",
    "PROPRIO_HISTORY_LENGTH",
    "PROPRIO_TOKEN_DIM",
    "REFERENCE_JOINT_NAMES",
    "REFERENCE_TO_ISAAC",
    "RgmtObservationBuilder",
    "RgmtPolicy",
    "RgmtReferenceFeatures",
    "quat_rotate_inverse_wxyz",
    "reference_features_from_qpos",
]
