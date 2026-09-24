"""Native ELF3 DWAQ policy for the BXI Mod runtime.

The exported DWAQ graph consumes five chronological 100 element observations
and returns one 29 element action vector.  The observation contract is kept
here, next to the model, so the runtime cannot accidentally feed an AMP-style
96 element frame to the graph.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from bxi_example_py_elf3.framework.inference.api import InferenceFrame
from bxi_example_py_elf3.framework.inference.contract import (
    PolicyJointContract,
)
from bxi_example_py_elf3.framework.inference.history import HistoryBuffer
from bxi_example_py_elf3.framework.inference.model import ModelSpec
from bxi_example_py_elf3.framework.inference.policy import JointPolicy
from bxi_example_py_elf3.framework.inference.runtime import (
    InferenceRuntime,
    default_runtime,
)
from bxi_example_py_elf3.framework.joints import JointLayout, JointParameterSet
from bxi_example_py_elf3.framework.mod_api.geometry import get_gravity_orientation


ELF3_DWAQ_JOINTS = JointLayout(
    (
        "waist_y_joint", "waist_x_joint", "waist_z_joint",
        "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
        "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
        "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
        "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
        "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint",
        "l_elbow_y_joint", "l_wrist_x_joint", "l_wrist_y_joint",
        "l_wrist_z_joint", "r_shoulder_y_joint", "r_shoulder_x_joint",
        "r_shoulder_z_joint", "r_elbow_y_joint", "r_wrist_x_joint",
        "r_wrist_y_joint", "r_wrist_z_joint",
    ),
    label="native ELF3 DWAQ",
)


_DEFAULT_POSITION = np.asarray(
    (0.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.6, -0.3, 0.0,
     -0.3, 0.0, 0.0, 0.6, -0.3, 0.0, 0.2, 0.2, 0.0, 0.6,
     0.0, 0.0, 0.0, 0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0),
    dtype=np.float32,
)
_ACTION_SCALE = np.asarray(
    (0.231, 0.154, 0.213, 0.213, 0.213, 0.231, 0.213, 0.373, 0.230,
     0.213, 0.213, 0.231, 0.213, 0.373, 0.230, 0.231, 0.231, 0.373,
     0.231, 0.373, 0.373, 0.373, 0.231, 0.231, 0.373, 0.231, 0.373,
     0.373, 0.373),
    dtype=np.float32,
)
_KP = np.asarray(
    (108.448, 162.672, 176.421, 176.421, 176.421, 54.224, 176.421,
     33.493, 21.771, 176.421, 176.421, 54.224, 176.421, 33.493, 21.771,
     54.224, 54.224, 16.747, 54.224, 16.747, 16.747, 16.747, 54.224,
     54.224, 16.747, 54.224, 16.747, 16.747, 16.747),
    dtype=np.float32,
)
_KD = np.asarray(
    (6.904, 10.356, 11.231, 11.231, 11.231, 3.452, 11.231, 2.132,
     1.386, 11.231, 11.231, 3.452, 11.231, 2.132, 1.386, 3.452, 3.452,
     1.066, 3.452, 1.066, 1.066, 1.066, 3.452, 3.452, 1.066, 3.452,
     1.066, 1.066, 1.066),
    dtype=np.float32,
)


ELF3_DWAQ_PARAMETERS = JointParameterSet.from_rows(
    ELF3_DWAQ_JOINTS,
    tuple(
        (name, float(default), float(kp), float(kd), float(scale))
        for name, default, kp, kd, scale in zip(
            ELF3_DWAQ_JOINTS.names,
            _DEFAULT_POSITION,
            _KP,
            _KD,
            _ACTION_SCALE,
        )
    ),
)

ELF3_DWAQ_COMMAND_MIN = np.asarray((-0.6, -0.5, -1.57), dtype=np.float32)
ELF3_DWAQ_COMMAND_MAX = np.asarray((1.0, 0.5, 1.57), dtype=np.float32)
ELF3_DWAQ_GAIT_PERIOD = 0.8
ELF3_DWAQ_GAIT_OFFSET = 0.5
ELF3_DWAQ_CONTROL_DT = 0.02
ELF3_DWAQ_OBSERVATION_CLIP = 100.0
ELF3_DWAQ_ACTION_CLIP = 100.0
ELF3_DWAQ_HISTORY_LENGTH = 5
ELF3_DWAQ_SINGLE_OBS_DIM = 100


def _validate_policy_metadata(model: object) -> None:
    """Reject a model whose adjacent ELF3 contract is not this policy's one.

    The training exporter writes ``policy.json`` next to the ONNX artifact.
    Keeping this check here prevents a model with a different observation
    history or joint contract from being accepted merely because it happens to
    have a compatible tensor shape.
    """

    if not isinstance(model, (str, Path)):
        return
    metadata_path = Path(model).with_name("policy.json")
    if not metadata_path.is_file():
        return
    try:
        metadata = json.loads(metadata_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read DWAQ policy metadata: {metadata_path}") from exc

    expected = {
        "num_obs": ELF3_DWAQ_SINGLE_OBS_DIM,
        "history_length": ELF3_DWAQ_HISTORY_LENGTH,
        "control_dt": ELF3_DWAQ_CONTROL_DT,
        "gait_period": ELF3_DWAQ_GAIT_PERIOD,
        "gait_offset": ELF3_DWAQ_GAIT_OFFSET,
        "policy_input_dim": ELF3_DWAQ_SINGLE_OBS_DIM * ELF3_DWAQ_HISTORY_LENGTH,
        "num_actions": ELF3_DWAQ_JOINTS.dof_num,
    }
    for key, value in expected.items():
        actual = metadata.get(key)
        if isinstance(value, float):
            valid = actual is not None and math.isclose(
                float(actual), value, abs_tol=1e-6
            )
        else:
            valid = actual == value
        if not valid:
            raise ValueError(
                f"DWAQ policy metadata mismatch for {key}: "
                f"expected {value!r}, got {actual!r} ({metadata_path})"
            )

    contract = metadata.get("contract")
    if not isinstance(contract, dict):
        raise ValueError(f"DWAQ policy metadata has no contract: {metadata_path}")
    if tuple(contract.get("joint_names", ())) != ELF3_DWAQ_JOINTS.names:
        raise ValueError(
            "DWAQ policy joint order does not match ELF3 contract: "
            f"{metadata_path}"
        )

    for field, expected_values in (
        ("default_joint_pos", _DEFAULT_POSITION),
        ("action_scale", _ACTION_SCALE),
        ("stiffness", _KP),
        ("damping", _KD),
    ):
        actual_values = [
            contract.get(field, {}).get(name) for name in ELF3_DWAQ_JOINTS.names
        ]
        # The deployment constants are stored as float32 while JSON keeps
        # decimal literals, so allow only the corresponding serialization
        # round-off (not a meaningful gain mismatch).
        if not np.allclose(actual_values, expected_values, rtol=0.0, atol=1e-4):
            raise ValueError(
                f"DWAQ policy {field} does not match ELF3 deployment contract: "
                f"{metadata_path}"
            )


class Elf3DwaqPolicy(JointPolicy):
    """Five-frame, 500-input ELF3 DWAQ actor exported to ONNX."""

    joint_contract = PolicyJointContract(ELF3_DWAQ_JOINTS, ELF3_DWAQ_JOINTS)

    def __init__(
        self,
        model: str | ModelSpec,
        *,
        runtime: InferenceRuntime | None = None,
        backend: str = "auto",
    ) -> None:
        super().__init__()
        _validate_policy_metadata(model)
        self._runtime = runtime or default_runtime()
        spec = model if isinstance(model, ModelSpec) else ModelSpec.portable_onnx(
            model, input_names=("history",), output_names=("actions",)
        )
        self._backend = self._runtime.open_backend(spec, backend=backend)
        input_shape = self._backend.input_shape("history")
        output_shape = self._backend.output_shape("actions")
        if input_shape[-1] != 500 or output_shape[-1] != 29:
            raise ValueError(
                f"ELF3 DWAQ expects history=500/actions=29, got "
                f"history={input_shape}, actions={output_shape}"
            )

        self._params = ELF3_DWAQ_PARAMETERS
        self._input = np.zeros((1, 500), dtype=np.float32)
        self._inputs = {"history": self._input}
        self._history = HistoryBuffer(
            ELF3_DWAQ_HISTORY_LENGTH,
            (ELF3_DWAQ_SINGLE_OBS_DIM,),
            dtype=np.float32,
        )
        self._previous_action = np.zeros(29, dtype=np.float32)
        self._single = np.zeros(100, dtype=np.float32)
        self._target = self._target_buffer.position
        self._estimated_velocity = np.zeros(3, dtype=np.float32)
        self._phase_time = 0.0
        self.publish_output(
            self._params.default_position,
            self._params.kp,
            self._params.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def _build_single(self, frame: InferenceFrame) -> np.ndarray:
        joints = self._joint_binding.bind(frame.joints)
        if frame.command is None:
            raise ValueError("Elf3DwaqPolicy requires a velocity command")

        self._single[:3] = np.asarray(frame.angular_velocity, dtype=np.float32)
        self._single[3:6] = get_gravity_orientation(frame.quat_wxyz)
        np.clip(
            np.asarray(frame.command, dtype=np.float32).reshape(3),
            ELF3_DWAQ_COMMAND_MIN,
            ELF3_DWAQ_COMMAND_MAX,
            out=self._single[6:9],
        )
        self._single[9:38] = joints.position - self._params.default_position
        self._single[38:67] = joints.velocity
        self._single[67:96] = self._previous_action

        left_phase = (self._phase_time % ELF3_DWAQ_GAIT_PERIOD) / ELF3_DWAQ_GAIT_PERIOD
        right_phase = (left_phase + ELF3_DWAQ_GAIT_OFFSET) % 1.0
        phases = np.asarray((left_phase, right_phase), dtype=np.float32)
        self._single[96:98] = np.sin(2.0 * math.pi * phases)
        self._single[98:100] = np.cos(2.0 * math.pi * phases)
        np.clip(
            self._single,
            -ELF3_DWAQ_OBSERVATION_CLIP,
            ELF3_DWAQ_OBSERVATION_CLIP,
            out=self._single,
        )
        return self._single

    def reset(self, frame: InferenceFrame) -> None:
        self._previous_action.fill(0.0)
        self._phase_time = 0.0
        self._history.fill(self._build_single(frame).copy())
        np.copyto(self._target, self._params.default_position)
        self.publish_output(
            self._target,
            self._params.kp,
            self._params.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def step(
        self,
        frame: InferenceFrame,
        dt: float,
        *,
        advance: bool = True,
    ):
        single = self._build_single(frame)
        if advance:
            self._history.append(single)
            self._history.write_into(self._input[0])
        else:
            self._history.preview_append_into(single, self._input[0])

        action = np.asarray(
            self._backend.run(self._inputs)["actions"], dtype=np.float32
        ).reshape(-1)
        if action.shape != (29,) or not np.all(np.isfinite(action)):
            raise ValueError(f"ELF3 DWAQ action is invalid: {action.shape}")
        np.clip(
            action,
            -ELF3_DWAQ_ACTION_CLIP,
            ELF3_DWAQ_ACTION_CLIP,
            out=action,
        )
        if advance:
            np.copyto(self._previous_action, action)
            # Training advances phase by one fixed 50 Hz step.  Do not use
            # wall-time or an overrun-adjusted dt here; that would feed a
            # phase sequence the actor never saw during training.
            self._phase_time += ELF3_DWAQ_CONTROL_DT

        np.add(
            self._params.default_position,
            action * self._params.action_scale,
            out=self._target,
        )
        return self.publish_output(
            self._target,
            self._params.kp,
            self._params.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def close(self) -> None:
        self._backend.close()
