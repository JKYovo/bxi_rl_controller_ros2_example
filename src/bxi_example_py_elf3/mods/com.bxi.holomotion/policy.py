"""HoloMotion observation and action decoding in the official BXI cycle."""

from __future__ import annotations

import math
from multiprocessing.connection import Connection
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from bxi_example_py_elf3.framework.inference import (
    InferenceFrame,
    JointPolicy,
    PolicyJointContract,
    PolicyOutput,
)
from bxi_example_py_elf3.framework.joints import JointLayout

from .reference import (
    WINDOW_SIZE,
    ReferenceReceiver,
    ReferenceWindow,
)

REFERENCE_JOINT_NAMES = (
    "waist_y_joint",
    "waist_x_joint",
    "waist_z_joint",
    "l_hip_y_joint",
    "l_hip_x_joint",
    "l_hip_z_joint",
    "l_knee_y_joint",
    "l_ankle_y_joint",
    "l_ankle_x_joint",
    "r_hip_y_joint",
    "r_hip_x_joint",
    "r_hip_z_joint",
    "r_knee_y_joint",
    "r_ankle_y_joint",
    "r_ankle_x_joint",
    "l_shoulder_y_joint",
    "l_shoulder_x_joint",
    "l_shoulder_z_joint",
    "l_elbow_y_joint",
    "l_wrist_x_joint",
    "l_wrist_y_joint",
    "l_wrist_z_joint",
    "r_shoulder_y_joint",
    "r_shoulder_x_joint",
    "r_shoulder_z_joint",
    "r_elbow_y_joint",
    "r_wrist_x_joint",
    "r_wrist_y_joint",
    "r_wrist_z_joint",
)

MODEL_JOINT_NAMES = (
    "l_hip_y_joint",
    "r_hip_y_joint",
    "waist_z_joint",
    "l_hip_x_joint",
    "r_hip_x_joint",
    "waist_x_joint",
    "l_hip_z_joint",
    "r_hip_z_joint",
    "waist_y_joint",
    "l_knee_y_joint",
    "r_knee_y_joint",
    "l_shoulder_y_joint",
    "r_shoulder_y_joint",
    "l_ankle_y_joint",
    "r_ankle_y_joint",
    "l_shoulder_x_joint",
    "r_shoulder_x_joint",
    "l_ankle_x_joint",
    "r_ankle_x_joint",
    "l_shoulder_z_joint",
    "r_shoulder_z_joint",
    "l_elbow_y_joint",
    "r_elbow_y_joint",
    "l_wrist_x_joint",
    "r_wrist_x_joint",
    "l_wrist_y_joint",
    "r_wrist_y_joint",
    "l_wrist_z_joint",
    "r_wrist_z_joint",
)

POLICY_DOF_SIGNS = np.asarray(
    (
        1,
        1,
        -1,
        1,
        1,
        -1,
        1,
        1,
        -1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
    ),
    dtype=np.float32,
)

MODEL_LAYOUT = JointLayout(MODEL_JOINT_NAMES, label="HoloMotion ELF3 29 DoF")
REFERENCE_TO_MODEL = np.asarray(
    tuple(REFERENCE_JOINT_NAMES.index(name) for name in MODEL_JOINT_NAMES),
    dtype=np.int64,
)
WAIST_MODEL_INDICES = np.asarray(
    tuple(MODEL_JOINT_NAMES.index(name) for name in REFERENCE_JOINT_NAMES[:3]),
    dtype=np.int64,
)
OBS_DIM = 604
ACTION_DIM = 29
ROOT_OFFSET_TORSO = np.asarray([0.0, 0.0, -0.2265], dtype=np.float32)
TIMING_REPORT_FRAMES = 500
ROPE_RESET_MARGIN = 64
WORKER_STARTUP_TIMEOUT_S = 60.0
WORKER_INFERENCE_TIMEOUT_S = 0.1


class LoggerLike(Protocol):
    def info(self, message: str) -> None: ...
    def warning(self, message: str) -> None: ...
    def error(self, message: str) -> None: ...


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    value = np.asarray(q, dtype=np.float32)
    norm = np.linalg.norm(value, axis=-1, keepdims=True)
    if np.any(norm < 1.0e-8) or not np.all(np.isfinite(norm)):
        raise ValueError("invalid quaternion")
    value = value / norm
    return np.where(value[..., :1] < 0.0, -value, value).astype(np.float32)


def quat_mul_wxyz(q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    q0 = np.asarray(q0, dtype=np.float32)
    q1 = np.asarray(q1, dtype=np.float32)
    w0, x0, y0, z0 = np.moveaxis(q0, -1, 0)
    w1, x1, y1, z1 = np.moveaxis(q1, -1, 0)
    out = np.empty(np.broadcast_shapes(q0.shape, q1.shape), dtype=np.float32)
    out[..., 0] = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    out[..., 1] = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    out[..., 2] = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    out[..., 3] = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    return out


def quat_conjugate_wxyz(q: np.ndarray) -> np.ndarray:
    out = np.asarray(q, dtype=np.float32).copy()
    out[..., 1:] *= -1.0
    return out


def quat_rotate_wxyz(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float32)
    value = np.asarray(vector, dtype=np.float32)
    qvec = quat[..., 1:]
    twice_cross = 2.0 * np.cross(qvec, value)
    return value + quat[..., :1] * twice_cross + np.cross(qvec, twice_cross)


def quat_rotate_inv_wxyz(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    return quat_rotate_wxyz(quat_conjugate_wxyz(q), vector)


def axis_quat(axis: int, angles: np.ndarray) -> np.ndarray:
    values = np.asarray(angles, dtype=np.float32)
    result = np.zeros(values.shape + (4,), dtype=np.float32)
    half = 0.5 * values
    result[..., 0] = np.cos(half)
    result[..., axis + 1] = np.sin(half)
    return result


def projected_gravity(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float32)
    qw, qx, qy, qz = np.moveaxis(quat, -1, 0)
    result = np.empty(quat.shape[:-1] + (3,), dtype=np.float32)
    result[..., 0] = 2.0 * (-qz * qx + qw * qy)
    result[..., 1] = -2.0 * (qz * qy + qw * qx)
    result[..., 2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return result


def yaw_from_quat(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float32)
    w, x, y, z = np.moveaxis(quat, -1, 0)
    return np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_quat(angle: float) -> np.ndarray:
    half = 0.5 * float(angle)
    return np.asarray(
        [math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float32
    )


def rot6d_from_quat(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float32)
    w, x, y, z = np.moveaxis(quat, -1, 0)
    matrix = np.empty(quat.shape[:-1] + (3, 3), dtype=np.float32)
    matrix[..., 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrix[..., 0, 1] = 2.0 * (x * y - z * w)
    matrix[..., 0, 2] = 2.0 * (x * z + y * w)
    matrix[..., 1, 0] = 2.0 * (x * y + z * w)
    matrix[..., 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrix[..., 1, 2] = 2.0 * (y * z - x * w)
    matrix[..., 2, 0] = 2.0 * (x * z - y * w)
    matrix[..., 2, 1] = 2.0 * (y * z + x * w)
    matrix[..., 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return matrix[..., :2].reshape(quat.shape[:-1] + (6,))


def _time_gradient(values: np.ndarray, fps: float = 50.0) -> np.ndarray:
    data = np.asarray(values, dtype=np.float32)
    if data.shape[0] < 2:
        raise ValueError("time gradient requires at least two frames")
    result = np.empty_like(data)
    result[0] = (data[1] - data[0]) * fps
    result[-1] = (data[-1] - data[-2]) * fps
    result[1:-1] = (data[2:] - data[:-2]) * (0.5 * fps)
    return result


def _quat_angular_velocity(
    quaternions: np.ndarray, fps: float = 50.0
) -> np.ndarray:
    quat = _normalize_quat(quaternions)
    if quat.shape[0] < 2:
        raise ValueError("angular velocity requires at least two quaternions")
    low = np.concatenate((quat[:1], quat[:-2], quat[-2:-1]), axis=0)
    high = np.concatenate((quat[1:2], quat[2:], quat[-1:]), axis=0)
    relative = _normalize_quat(quat_mul_wxyz(high, quat_conjugate_wxyz(low)))
    negative = relative[:, 0] < 0.0
    relative[negative] *= -1.0
    xyz = relative[:, 1:]
    magnitude = np.linalg.norm(xyz, axis=1)
    half_angle = np.arctan2(magnitude, relative[:, 0])
    angle = 2.0 * half_angle
    scale = np.empty_like(angle)
    small = np.abs(angle) <= 1.0e-6
    scale[small] = 2.0
    scale[~small] = angle[~small] / magnitude[~small]
    inverse_dt = np.full(quat.shape[0], 0.5 * fps, dtype=np.float32)
    inverse_dt[[0, -1]] = fps
    return (xyz * scale[:, None] * inverse_dt[:, None]).astype(np.float32)


def semantic_root_from_torso(
    torso_quat_wxyz: np.ndarray,
    torso_ang_vel_local: np.ndarray,
    waist_pos_yxz: np.ndarray,
    waist_vel_yxz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    torso = _normalize_quat(np.asarray(torso_quat_wxyz).reshape(4))
    qy, qx, qz = np.asarray(waist_pos_yxz, dtype=np.float32).reshape(3)
    vy, vx, vz = np.asarray(waist_vel_yxz, dtype=np.float32).reshape(3)
    torso_y = quat_mul_wxyz(torso, axis_quat(1, qy))
    torso_yx = quat_mul_wxyz(torso_y, axis_quat(0, qx))
    root = _normalize_quat(quat_mul_wxyz(torso_yx, axis_quat(2, qz)))
    omega_world = quat_rotate_wxyz(torso, torso_ang_vel_local)
    omega_world += quat_rotate_wxyz(torso, np.asarray([0.0, vy, 0.0]))
    omega_world += quat_rotate_wxyz(torso_y, np.asarray([vx, 0.0, 0.0]))
    omega_world += quat_rotate_wxyz(torso_yx, np.asarray([0.0, 0.0, vz]))
    return root, quat_rotate_inv_wxyz(root, omega_world).astype(np.float32)


@dataclass(frozen=True, slots=True)
class ReferenceFeatures:
    dof_pos_model_signed: np.ndarray
    root_position: np.ndarray
    root_quat_wxyz: np.ndarray
    root_linear_velocity_local: np.ndarray
    root_angular_velocity_local: np.ndarray
    gravity: np.ndarray


def reference_features_from_qpos(qpos: np.ndarray) -> ReferenceFeatures:
    data = np.asarray(qpos, dtype=np.float32)
    if data.shape != (WINDOW_SIZE, 36):
        raise ValueError(
            f"reference window must have shape ({WINDOW_SIZE}, 36), got {data.shape}"
        )
    if not np.all(np.isfinite(data)):
        raise ValueError("reference window contains NaN or Inf")
    torso_position = data[:, :3]
    torso_quat = _normalize_quat(data[:, 3:7])
    dof_reference = data[:, 7:]
    waist = dof_reference[:, :3]
    torso_y = quat_mul_wxyz(torso_quat, axis_quat(1, waist[:, 0]))
    torso_yx = quat_mul_wxyz(torso_y, axis_quat(0, waist[:, 1]))
    root_quat = _normalize_quat(
        quat_mul_wxyz(torso_yx, axis_quat(2, waist[:, 2]))
    )
    root_position = torso_position + quat_rotate_wxyz(
        torso_quat,
        np.broadcast_to(ROOT_OFFSET_TORSO, torso_position.shape),
    )
    # Legacy online contract: the policy consumes all 11 frames. Interior
    # velocities use centered differences; the two endpoints use one-sided
    # differences because no extra past/tail reference frames are buffered.
    observed_root_quat = root_quat
    root_velocity_world = _time_gradient(root_position)
    root_omega_world = _quat_angular_velocity(root_quat)
    return ReferenceFeatures(
        dof_pos_model_signed=(
            dof_reference[:, REFERENCE_TO_MODEL] * POLICY_DOF_SIGNS[None, :]
        ).astype(np.float32),
        root_position=root_position.astype(np.float32),
        root_quat_wxyz=observed_root_quat.astype(np.float32),
        root_linear_velocity_local=quat_rotate_inv_wxyz(
            observed_root_quat, root_velocity_world
        ).astype(np.float32),
        root_angular_velocity_local=quat_rotate_inv_wxyz(
            observed_root_quat, root_omega_world
        ).astype(np.float32),
        gravity=projected_gravity(observed_root_quat),
    )


class HoloMotionObservationBuilder:
    def __init__(self, default_joint_position: np.ndarray) -> None:
        self.default_joint_position = np.asarray(
            default_joint_position, dtype=np.float32
        ).reshape(ACTION_DIM)
        self.obs = np.zeros((1, OBS_DIM), dtype=np.float32)
        self._yaw_alignment = np.asarray(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float32
        )
        self._yaw_ready = False

    def reset(self) -> None:
        self.obs.fill(0.0)
        self._yaw_alignment[:] = (1.0, 0.0, 0.0, 0.0)
        self._yaw_ready = False

    def _align(self, reference_quat: np.ndarray) -> np.ndarray:
        return _normalize_quat(
            quat_mul_wxyz(self._yaw_alignment, reference_quat)
        )

    def build(
        self,
        *,
        features: ReferenceFeatures,
        robot_joint_position: np.ndarray,
        robot_joint_velocity: np.ndarray,
        robot_root_quat: np.ndarray,
        robot_root_angular_velocity: np.ndarray,
        last_action: np.ndarray,
    ) -> np.ndarray:
        robot_quat = _normalize_quat(robot_root_quat)
        if not self._yaw_ready:
            offset = float(yaw_from_quat(robot_quat)) - float(
                yaw_from_quat(features.root_quat_wxyz[0])
            )
            self._yaw_alignment[:] = yaw_quat(offset)
            self._yaw_ready = True

        ref_current = features.root_quat_wxyz[0]
        ref_future = features.root_quat_wxyz[1:]
        aligned_current = self._align(ref_current)
        aligned_future = self._align(ref_future)
        yaw_error = float(yaw_from_quat(aligned_current)) - float(
            yaw_from_quat(robot_quat)
        )
        future_yaw_delta = yaw_from_quat(ref_future) - float(
            yaw_from_quat(ref_current)
        )
        relative_future = _normalize_quat(
            quat_mul_wxyz(
                quat_conjugate_wxyz(robot_quat)[None, :], aligned_future
            )
        )
        robot_position = (
            np.asarray(robot_joint_position, dtype=np.float32)
            - self.default_joint_position
        ) * POLICY_DOF_SIGNS
        robot_velocity = (
            np.asarray(robot_joint_velocity, dtype=np.float32)
            * POLICY_DOF_SIGNS
        )
        terms = (
            features.gravity[0],
            features.root_linear_velocity_local[0],
            features.root_angular_velocity_local[0],
            features.dof_pos_model_signed[0],
            features.root_position[0, 2:3],
            np.asarray([math.sin(yaw_error), math.cos(yaw_error)]),
            projected_gravity(robot_quat),
            np.asarray(robot_root_angular_velocity, dtype=np.float32),
            robot_position,
            robot_velocity,
            np.asarray(last_action, dtype=np.float32),
            features.dof_pos_model_signed[1:].reshape(-1),
            features.root_position[1:, 2].reshape(-1),
            features.gravity[1:].reshape(-1),
            features.root_linear_velocity_local[1:].reshape(-1),
            features.root_angular_velocity_local[1:].reshape(-1),
            np.stack(
                (np.sin(future_yaw_delta), np.cos(future_yaw_delta)), axis=1
            ).reshape(-1),
            rot6d_from_quat(relative_future).reshape(-1),
        )
        flat = np.concatenate(terms).astype(np.float32, copy=False)
        if flat.shape != (OBS_DIM,) or not np.all(np.isfinite(flat)):
            raise ValueError(f"invalid HoloMotion observation: {flat.shape}")
        self.obs[0] = flat
        return self.obs


def _metadata_floats(metadata: dict[str, str], key: str) -> np.ndarray:
    if key not in metadata:
        raise ValueError(f"ONNX metadata is missing {key}")
    return np.asarray(
        [float(value) for value in metadata[key].split(",") if value],
        dtype=np.float32,
    )


class _OnnxWorkerClient:
    """Own a Mod-local inference subprocess outside the controller GIL."""

    def __init__(self, model_path: Path) -> None:
        parent_socket, child_socket = socket.socketpair()
        child_fd = child_socket.detach()
        self.connection = Connection(parent_socket.detach())
        self.process: subprocess.Popen[bytes] | None = None
        self.closed = False
        worker_path = Path(__file__).with_name("inference_worker.py")
        command = (
            sys.executable,
            "-u",
            str(worker_path),
            "--connection-fd",
            str(child_fd),
            "--model",
            str(model_path),
        )
        try:
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
                pass_fds=(child_fd,),
            )
        except BaseException:
            self.connection.close()
            self.closed = True
            raise
        finally:
            os.close(child_fd)
        if not self.connection.poll(WORKER_STARTUP_TIMEOUT_S):
            self.close()
            raise TimeoutError(
                "HoloMotion inference worker did not initialize within "
                f"{WORKER_STARTUP_TIMEOUT_S:.1f}s"
            )
        try:
            message = self.connection.recv()
        except EOFError as exc:
            return_code = self.process.poll()
            self.close()
            raise RuntimeError(
                "HoloMotion inference worker exited before initialization "
                f"(return code {return_code})"
            ) from exc
        if not isinstance(message, dict) or not message.get("ok", False):
            detail = (
                message.get("error", "unknown worker initialization error")
                if isinstance(message, dict)
                else repr(message)
            )
            self.close()
            raise RuntimeError(f"HoloMotion inference worker failed: {detail}")
        self.providers = tuple(message["providers"])
        self.metadata = dict(message["metadata"])
        self.cold_warmup_ms = float(message["cold_warmup_ms"])

    def run(
        self,
        observation: np.ndarray,
        kv_cache: np.ndarray,
        step_index: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        process = self.process
        if self.closed or process is None or process.poll() is not None:
            raise RuntimeError("HoloMotion inference worker is not running")
        self.connection.send(
            {
                "op": "infer",
                "obs": np.asarray(observation, dtype=np.float32),
                "past_key_values": np.asarray(kv_cache, dtype=np.float32),
                "step_idx": int(step_index),
            }
        )
        if not self.connection.poll(WORKER_INFERENCE_TIMEOUT_S):
            raise TimeoutError(
                "HoloMotion inference worker exceeded "
                f"{WORKER_INFERENCE_TIMEOUT_S * 1000.0:.0f}ms"
            )
        try:
            message = self.connection.recv()
        except EOFError as exc:
            raise RuntimeError(
                "HoloMotion inference worker exited during inference"
            ) from exc
        if not isinstance(message, dict) or not message.get("ok", False):
            detail = (
                message.get("error", "unknown inference error")
                if isinstance(message, dict)
                else repr(message)
            )
            raise RuntimeError(f"HoloMotion worker inference failed: {detail}")
        return (
            np.asarray(message["actions"], dtype=np.float32),
            np.asarray(message["present_key_values"], dtype=np.float32),
        )

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        process = self.process
        try:
            if process is not None and process.poll() is None:
                try:
                    self.connection.send({"op": "close"})
                except (BrokenPipeError, EOFError, OSError):
                    pass
        finally:
            self.connection.close()
        if process is None:
            return
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)


class HoloMotionPolicy(JointPolicy):
    joint_contract = PolicyJointContract(
        observation=MODEL_LAYOUT,
        action=MODEL_LAYOUT,
    )

    def __init__(
        self,
        *,
        model_path: Path,
        receiver: ReferenceReceiver,
    ) -> None:
        super().__init__()
        self.model_path = Path(model_path).resolve()
        self.receiver = receiver
        self.worker: _OnnxWorkerClient | None = _OnnxWorkerClient(
            self.model_path
        )
        try:
            self.providers = self.worker.providers
            self.cold_warmup_ms = self.worker.cold_warmup_ms
            metadata = self.worker.metadata
            joint_names = tuple(
                value
                for value in metadata.get("joint_names", "").split(",")
                if value
            )
            if joint_names != MODEL_JOINT_NAMES:
                raise ValueError(
                    "HoloMotion ONNX joint order does not match the ELF3 contract"
                )
            self.action_scale = _metadata_floats(metadata, "action_scale")
            self.default_joint_position = _metadata_floats(
                metadata, "default_joint_pos"
            )
            self.kp = _metadata_floats(metadata, "joint_stiffness")
            self.kd = _metadata_floats(metadata, "joint_damping")
            for name, array in (
                ("action_scale", self.action_scale),
                ("default_joint_pos", self.default_joint_position),
                ("kp", self.kp),
                ("kd", self.kd),
            ):
                if array.shape != (ACTION_DIM,) or not np.all(
                    np.isfinite(array)
                ):
                    raise ValueError(f"invalid ONNX {name} metadata")
            self.rope_max_seq_len = int(
                metadata.get("rope_max_seq_len", "8192")
            )
            if self.rope_max_seq_len <= 0:
                raise ValueError("rope_max_seq_len must be positive")
        except BaseException:
            self.close()
            raise
        self.observation = HoloMotionObservationBuilder(
            self.default_joint_position
        )
        self.kv_cache = np.zeros((1, 2, 1, 32, 4, 64), dtype=np.float32)
        self.step_index = 0
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.started = False
        self.should_exit_to_normal = False
        self.status = "ready"
        self.last_status = "waiting_reference"
        self.last_inference_ms = 0.0
        self._timing_samples = np.empty(
            (TIMING_REPORT_FRAMES, 6), dtype=np.float64
        )
        self._timing_sample_count = 0
        self._reference_lag_sample_count = 0
        self._reference_backlog_sum = 0
        self._reference_backlog_max = 0
        self.last_reference_frame = -1
        self._stream_epoch: int | None = None
        self._logger: LoggerLike | None = None
        self._last_reported_status: str | None = None
        self.publish_output(self.default_joint_position, self.kp, self.kd)

    def close(self) -> None:
        """Terminate the Mod-local worker and release its ONNX session."""

        worker = getattr(self, "worker", None)
        self.worker = None
        if worker is not None:
            worker.close()

    def _record_timing(self, values_ms: tuple[float, ...]) -> None:
        index = self._timing_sample_count
        self._timing_samples[index] = values_ms
        self._timing_sample_count = index + 1
        if self._timing_sample_count < TIMING_REPORT_FRAMES:
            return
        if self._logger is not None:
            names = (
                "robot",
                "reference",
                "observation",
                "onnx",
                "decode",
                "total",
            )
            sample = self._timing_samples
            parts = []
            for column, name in enumerate(names):
                values = sample[:, column]
                parts.append(
                    f"{name}=P50 {np.percentile(values, 50):.2f}/"
                    f"P95 {np.percentile(values, 95):.2f}/"
                    f"max {np.max(values):.2f} ms"
                )
            self._logger.info(
                "HoloMotion 500-frame timing (P50/P95/max): "
                + "; ".join(parts)
            )
        self._timing_sample_count = 0

    def bind_logger(self, logger: LoggerLike) -> None:
        self._logger = logger

    @property
    def stream_epoch(self) -> int | None:
        return self._stream_epoch

    def _record_reference_lag(self, window: ReferenceWindow) -> None:
        """Report ordered-playout backlog without per-frame log I/O."""

        backlog_frames = max(
            0,
            int(window.source_latest_frame_index)
            - int(window.latest_frame_index),
        )
        self._reference_lag_sample_count += 1
        self._reference_backlog_sum += backlog_frames
        self._reference_backlog_max = max(
            self._reference_backlog_max, backlog_frames
        )
        if self._reference_lag_sample_count < 50:
            return
        count = self._reference_lag_sample_count
        current_lag_frames = max(
            0,
            int(window.source_latest_frame_index) - int(window.frame_index),
        )
        if self._logger is not None:
            self._logger.info(
                "HoloMotion reference lag (50-step): "
                f"current={window.frame_index}, "
                f"window_latest={window.latest_frame_index}, "
                f"source_latest={window.source_latest_frame_index}; "
                f"current_lag={current_lag_frames} frames/"
                f"~{current_lag_frames * 20} ms; "
                f"extra_backlog=last {backlog_frames}/"
                f"mean {self._reference_backlog_sum / count:.2f}/"
                f"max {self._reference_backlog_max} frames; "
                f"window_tail_age={window.receive_age_s * 1000.0:.1f} ms; "
                "source_latest_age="
                f"{window.source_latest_receive_age_s * 1000.0:.1f} ms"
            )
        self._reference_lag_sample_count = 0
        self._reference_backlog_sum = 0
        self._reference_backlog_max = 0

    def _set_status(self, value: str) -> None:
        self.last_status = value
        if value == self._last_reported_status or self._logger is None:
            return
        self._last_reported_status = value
        if value == "reference_stale":
            self._logger.warning(f"HoloMotion status: {value}")
        else:
            self._logger.info(f"HoloMotion status: {value}")

    def reset(self, frame: InferenceFrame | None = None) -> None:
        self.kv_cache.fill(0.0)
        self.last_action.fill(0.0)
        self.step_index = 0
        self.started = False
        self.should_exit_to_normal = False
        self.last_reference_frame = -1
        self._stream_epoch = None
        self.observation.reset()
        self._timing_sample_count = 0
        self._reference_lag_sample_count = 0
        self._reference_backlog_sum = 0
        self._reference_backlog_max = 0
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
        age = self.receiver.latest_receive_age()
        if self.started and age > self.receiver.max_source_age_s:
            self.should_exit_to_normal = True
            self._set_status("reference_stale")
        else:
            self._set_status("waiting_reference")
        return None

    def _maybe_reset_rope_window(self) -> bool:
        """Cycle the exported RoPE window without leaving teleoperation."""

        reset_at = self.rope_max_seq_len - ROPE_RESET_MARGIN
        if reset_at <= 0:
            reset_at = self.rope_max_seq_len
        if self.step_index < reset_at:
            return False

        old_step_index = self.step_index
        self.kv_cache.fill(0.0)
        self.step_index = 0
        if self._logger is not None:
            self._logger.warning(
                "HoloMotion RoPE step index reached "
                f"{old_step_index}/{self.rope_max_seq_len}; reset KV cache and "
                "step index while keeping tracking active"
            )
        return True

    def step(
        self,
        frame: InferenceFrame,
        dt: float,
        *,
        advance: bool = True,
    ) -> PolicyOutput:
        if not advance:
            return self.output
        window = self._reference_or_wait()
        if window is None:
            return self.output
        if (
            self._stream_epoch is not None
            and window.stream_epoch != self._stream_epoch
        ):
            self.kv_cache.fill(0.0)
            self.last_action.fill(0.0)
            self.step_index = 0
            self.observation.reset()
            self.started = False
        self._stream_epoch = window.stream_epoch
        step_started_at = time.perf_counter()
        joints = self.bind_joints(frame)
        semantic_quat, semantic_omega = semantic_root_from_torso(
            frame.quat_wxyz,
            frame.angular_velocity,
            joints.position[WAIST_MODEL_INDICES],
            joints.velocity[WAIST_MODEL_INDICES],
        )
        robot_ready_at = time.perf_counter()
        features = reference_features_from_qpos(window.qpos)
        reference_ready_at = time.perf_counter()
        observation = self.observation.build(
            features=features,
            robot_joint_position=joints.position,
            robot_joint_velocity=joints.velocity,
            robot_root_quat=semantic_quat,
            robot_root_angular_velocity=semantic_omega,
            last_action=self.last_action,
        )
        observation_ready_at = time.perf_counter()
        worker = self.worker
        if worker is None:
            raise RuntimeError("HoloMotion policy worker is closed")
        self._maybe_reset_rope_window()
        actions, present = worker.run(
            observation,
            self.kv_cache,
            self.step_index,
        )
        inference_ready_at = time.perf_counter()
        self.last_inference_ms = (
            inference_ready_at - observation_ready_at
        ) * 1000.0
        action = np.asarray(actions, dtype=np.float32).reshape(ACTION_DIM)
        if not np.all(np.isfinite(action)):
            raise ValueError("HoloMotion action contains NaN or Inf")
        targets = action * self.action_scale + self.default_joint_position
        if not np.all(np.isfinite(targets)):
            raise ValueError("HoloMotion target contains NaN or Inf")
        self.kv_cache = np.asarray(present, dtype=np.float32)
        self.last_action[:] = action
        self.receiver.advance_after_successful_step(window.frame_index)
        self._record_reference_lag(window)
        self.last_reference_frame = window.frame_index
        self.step_index += 1
        self.started = True
        self._set_status("tracking")
        result = self.publish_output(targets, self.kp, self.kd)
        step_finished_at = time.perf_counter()
        self._record_timing(
            (
                (robot_ready_at - step_started_at) * 1000.0,
                (reference_ready_at - robot_ready_at) * 1000.0,
                (observation_ready_at - reference_ready_at) * 1000.0,
                (inference_ready_at - observation_ready_at) * 1000.0,
                (step_finished_at - inference_ready_at) * 1000.0,
                (step_finished_at - step_started_at) * 1000.0,
            )
        )
        return result

    def decode_into(self, outputs) -> None:
        raise NotImplementedError(
            "HoloMotionPolicy owns its recurrent ONNX step"
        )


__all__ = [
    "MODEL_JOINT_NAMES",
    "POLICY_DOF_SIGNS",
    "REFERENCE_JOINT_NAMES",
    "HoloMotionObservationBuilder",
    "HoloMotionPolicy",
    "ReferenceFeatures",
    "reference_features_from_qpos",
    "semantic_root_from_torso",
]
