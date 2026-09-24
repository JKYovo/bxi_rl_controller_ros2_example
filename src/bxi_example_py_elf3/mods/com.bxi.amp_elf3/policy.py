"""Native Amp_mjlab ELF3 AMP policy for the Mod runtime."""

from __future__ import annotations

import numpy as np

from bxi_example_py_elf3.framework.inference.api import InferenceFrame
from bxi_example_py_elf3.framework.inference.contract import (
    JointInputBinding,
    PolicyJointContract,
)
from bxi_example_py_elf3.framework.inference.history import HistoryBuffer
from bxi_example_py_elf3.framework.inference.model import ModelSpec
from bxi_example_py_elf3.framework.inference.policy import JointPolicy
from bxi_example_py_elf3.framework.inference.runtime import (
    InferenceRuntime,
    default_runtime,
)
from bxi_example_py_elf3.framework.joints import (
    JointLayout,
    JointParameterSet,
    JointTargetView,
)
from bxi_example_py_elf3.framework.mod_api.geometry import get_gravity_orientation


ELF3_AMP_JOINTS = JointLayout(
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
    label="native Amp_mjlab ELF3 AMP",
)


def _parameters() -> JointParameterSet:
    rows = {
        "waist_y_joint": (0.0, 108.448, 6.904, 0.231),
        "waist_x_joint": (0.0, 162.672, 10.356, 0.154),
        "waist_z_joint": (0.0, 176.421, 11.231, 0.213),
        "l_hip_y_joint": (-0.3, 176.421, 11.231, 0.213),
        "l_hip_x_joint": (0.0, 176.421, 11.231, 0.213),
        "l_hip_z_joint": (0.0, 54.224, 3.452, 0.231),
        "l_knee_y_joint": (0.6, 176.421, 11.231, 0.213),
        "l_ankle_y_joint": (-0.3, 33.493, 2.132, 0.373),
        "l_ankle_x_joint": (0.0, 21.771, 1.386, 0.230),
        "r_hip_y_joint": (-0.3, 176.421, 11.231, 0.213),
        "r_hip_x_joint": (0.0, 176.421, 11.231, 0.213),
        "r_hip_z_joint": (0.0, 54.224, 3.452, 0.231),
        "r_knee_y_joint": (0.6, 176.421, 11.231, 0.213),
        "r_ankle_y_joint": (-0.3, 33.493, 2.132, 0.373),
        "r_ankle_x_joint": (0.0, 21.771, 1.386, 0.230),
        "l_shoulder_y_joint": (0.2, 54.224, 3.452, 0.231),
        "l_shoulder_x_joint": (0.2, 54.224, 3.452, 0.231),
        "l_shoulder_z_joint": (0.0, 16.747, 1.066, 0.373),
        "l_elbow_y_joint": (0.6, 54.224, 3.452, 0.231),
        "l_wrist_x_joint": (0.0, 16.747, 1.066, 0.373),
        "l_wrist_y_joint": (0.0, 16.747, 1.066, 0.373),
        "l_wrist_z_joint": (0.0, 16.747, 1.066, 0.373),
        "r_shoulder_y_joint": (0.2, 54.224, 3.452, 0.231),
        "r_shoulder_x_joint": (-0.2, 54.224, 3.452, 0.231),
        "r_shoulder_z_joint": (0.0, 16.747, 1.066, 0.373),
        "r_elbow_y_joint": (0.6, 54.224, 3.452, 0.231),
        "r_wrist_x_joint": (0.0, 16.747, 1.066, 0.373),
        "r_wrist_y_joint": (0.0, 16.747, 1.066, 0.373),
        "r_wrist_z_joint": (0.0, 16.747, 1.066, 0.373),
    }
    return JointParameterSet.from_rows(
        ELF3_AMP_JOINTS,
        tuple((name, *rows[name]) for name in ELF3_AMP_JOINTS.names),
    )


ELF3_AMP_PARAMETERS = _parameters()

# V4 command envelope in physical units (m/s, m/s, rad/s). Observation 6:9
# receives these clipped velocities directly, without normalization or heading.
ELF3_AMP_COMMAND_MIN = np.asarray((-0.6, -1.0, -1.0), dtype=np.float32)
ELF3_AMP_COMMAND_MAX = np.asarray((1.0, 1.0, 2.0), dtype=np.float32)


class Elf3AmpPolicy(JointPolicy):
    """4-frame, 384-input AMP actor exported by Amp_mjlab."""

    joint_contract = PolicyJointContract(ELF3_AMP_JOINTS, ELF3_AMP_JOINTS)

    def __init__(
        self,
        model: str | ModelSpec,
        *,
        runtime: InferenceRuntime | None = None,
        backend: str = "auto",
    ) -> None:
        super().__init__()
        self._runtime = runtime or default_runtime()
        spec = model if isinstance(model, ModelSpec) else ModelSpec.portable_onnx(
            model, input_names=("obs",), output_names=("actions",)
        )
        self._backend = self._runtime.open_backend(spec, backend=backend)
        input_shape = self._backend.input_shape("obs")
        output_shape = self._backend.output_shape("actions")
        if input_shape[-1] != 384 or output_shape[-1] != 29:
            raise ValueError(
                f"ELF3 AMP expects obs=384/actions=29, got "
                f"obs={input_shape}, actions={output_shape}"
            )

        self._params = ELF3_AMP_PARAMETERS
        self._input = np.zeros((1, 384), dtype=np.float32)
        self._inputs = {"obs": self._input}
        self._history = HistoryBuffer(4, (96,), dtype=np.float32)
        self._previous_action = np.zeros(29, dtype=np.float32)
        self._single = np.zeros(96, dtype=np.float32)
        self._target = self._target_buffer.position
        self._estimated_velocity = np.zeros(3, dtype=np.float32)
        self.publish_output(
            self._params.default_position,
            self._params.kp,
            self._params.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def _build_single(self, frame: InferenceFrame) -> np.ndarray:
        joints = self._joint_binding.bind(frame.joints)
        command = frame.command
        if command is None:
            raise ValueError("Elf3AmpPolicy requires a velocity command")
        self._single[:3] = np.asarray(frame.angular_velocity, dtype=np.float32)
        self._single[3:6] = get_gravity_orientation(frame.quat_wxyz)
        command_array = np.asarray(command, dtype=np.float32).reshape(3)
        np.clip(
            command_array,
            ELF3_AMP_COMMAND_MIN,
            ELF3_AMP_COMMAND_MAX,
            out=self._single[6:9],
        )
        self._single[9:38] = joints.position - self._params.default_position
        self._single[38:67] = joints.velocity
        self._single[67:96] = self._previous_action
        return self._single

    def reset(self, frame: InferenceFrame) -> None:
        self._previous_action.fill(0.0)
        self._history.fill(self._build_single(frame).copy())
        np.copyto(self._target, self._params.default_position)
        self.publish_output(
            self._target, self._params.kp, self._params.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def step(self, frame: InferenceFrame, dt: float, *, advance: bool = True):
        single = self._build_single(frame)
        if advance:
            self._history.append(single)
            self._history.write_into(self._input[0])
        else:
            self._history.preview_append_into(single, self._input[0])
        action = np.asarray(
            self._backend.run(self._inputs)["actions"], dtype=np.float32
        ).reshape(-1)
        if action.shape != (29,):
            raise ValueError(f"AMP action shape is {action.shape}, expected (29,)")
        if advance:
            np.copyto(self._previous_action, action)
        np.add(
            self._params.default_position,
            action * self._params.action_scale,
            out=self._target,
        )
        return self.publish_output(
            self._target, self._params.kp, self._params.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def close(self) -> None:
        self._backend.close()
