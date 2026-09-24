from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import numpy as np

from bxi_example_py_elf3.framework.inference.api import InferenceFrame, PolicyOutput
from bxi_example_py_elf3.framework.inference.contract import PolicyJointContract
from bxi_example_py_elf3.framework.inference.history import HistoryBuffer
from bxi_example_py_elf3.framework.inference.model import ModelSpec
from bxi_example_py_elf3.framework.inference.policy import JointPolicy
from bxi_example_py_elf3.framework.inference.runtime import InferenceRuntime, default_runtime
from bxi_example_py_elf3.framework.mod_api.geometry import get_gravity_orientation
from bxi_example_py_elf3.policies.joints import ELF3_ISAAC_PARAMETERS


# Public names are retained for the existing Mod/state ABI.  The new export is
# one policy: there is no specialist/motion/brake selection or mirroring.
ACTOR_ROLES = ("policy",)
POLICY_JOINT_NAMES = ELF3_ISAAC_PARAMETERS.layout.names
INPUT_DIM = 1020
SINGLE_FRAME_DIM = 102
HISTORY_FRAMES = 10
ACTION_DIM = 29
POLICY_DT_S = 0.02
GAIT_CYCLE_S = 0.85
BRAKE_DURATION_S = 3.0
WAIST_X_INDEX = POLICY_JOINT_NAMES.index("waist_x_joint")
WAIST_X_SAFE_ABS_RAD = 0.2618 * 0.90 - 0.01


def classify_commissioning_command(command: object) -> str:
    value = np.asarray(command, dtype=np.float32).reshape(3)
    if not np.all(np.isfinite(value)):
        raise ValueError("command contains NaN or Inf")
    if np.max(np.abs(value), initial=0.0) <= 1.0e-4:
        return "zero"
    active = np.flatnonzero(np.abs(value) > 1.0e-4)
    if active.size != 1:
        raise ValueError("combined commands are outside the commissioning card")
    axis = int(active[0])
    actual = float(value[axis])
    expected = {
        (0, 1): ("forward", 0.375),
        (0, -1): ("backward", -0.25),
        (1, 1): ("left", 0.20),
        (1, -1): ("right", -0.20),
        (2, 1): ("yaw_left", 0.40),
        (2, -1): ("yaw_right", -0.40),
    }[(axis, 1 if actual > 0.0 else -1)]
    if abs(actual - expected[1]) > 1.0e-5:
        raise ValueError(
            f"{expected[0]} command {actual:.6f} is outside the evaluated value "
            f"{expected[1]:.6f}"
        )
    return expected[0]


class Elf3ThreeActorRouterPolicy(JointPolicy):
    """Single-policy adapter for the TienKung-Lab AMP ONNX export.

    The class name remains stable so existing state and resource identifiers do
    not change.  A legacy mapping is accepted only as a migration convenience;
    exactly one artifact is opened and used.
    """

    joint_contract = PolicyJointContract(
        observation=ELF3_ISAAC_PARAMETERS.layout,
        action=ELF3_ISAAC_PARAMETERS.layout,
    )

    def __init__(
        self,
        model: str | Path | ModelSpec | Mapping[str, str | Path | ModelSpec],
        *,
        runtime: InferenceRuntime | None = None,
        backend: str = "auto",
    ) -> None:
        super().__init__()
        if isinstance(model, Mapping):
            if "policy" in model:
                model = model["policy"]
            elif len(model) == 1:
                model = next(iter(model.values()))
            else:
                raise ValueError("single-policy Mod requires one model artifact")
        spec = model if isinstance(model, ModelSpec) else ModelSpec.portable_onnx(
            model, input_names=("obs",), output_names=("actions",)
        )
        self._runtime = runtime or default_runtime()
        self._closed = False
        try:
            self._backend = self._runtime.open_backend(spec, backend=backend)
            if tuple(self._backend.input_shape("obs"))[-1] != INPUT_DIM:
                raise ValueError(f"policy input ABI is not {INPUT_DIM}")
            if int(self._backend.output_shape("actions")[-1]) != ACTION_DIM:
                raise ValueError(f"policy output ABI is not {ACTION_DIM}")
        except BaseException:
            self.close()
            raise

        self.num_obs = INPUT_DIM
        self.single_obs_dim = SINGLE_FRAME_DIM
        self.obs_history_len = HISTORY_FRAMES
        self._parameters = ELF3_ISAAC_PARAMETERS
        self._history = HistoryBuffer(HISTORY_FRAMES, (SINGLE_FRAME_DIM,), dtype=np.float32)
        self._single_obs = np.zeros(SINGLE_FRAME_DIM, dtype=np.float32)
        self._obs = np.zeros(INPUT_DIM, dtype=np.float32)
        self._inputs = {"obs": self._obs.reshape(1, -1)}
        self._action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._previous_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._previous_action_checkpoint = np.zeros_like(self._previous_action)
        self._scaled_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._target = self._target_buffer.position
        self._estimated_velocity = np.zeros(3, dtype=np.float32)
        self._route_command = np.zeros(3, dtype=np.float32)
        self._episode_length = 0
        self._gait_phase = np.zeros(2, dtype=np.float32)
        self._phase_ratio = np.asarray((0.38, 0.38), dtype=np.float32)
        self._phase_offset = np.asarray((0.38, 0.88), dtype=np.float32)
        self._mode = "HOLD"
        # MOVE/BRAKE/HOLD are diagnostic modes only.  The backend role is
        # always the single TienKung AMP policy.
        self._role = "policy"
        self._user_active = False
        self._brake_elapsed_s = 0.0
        np.copyto(self._target, self._parameters.default_position)
        self.publish_output(
            self._target, self._parameters.kp, self._parameters.kd,
            estimated_velocity=self._estimated_velocity,
        )

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def role(self) -> str:
        return self._role

    def reset(self, frame: InferenceFrame) -> None:
        joints = self.bind_joints(frame)
        command = np.asarray(frame.command, dtype=np.float32).reshape(3)
        command_class = classify_commissioning_command(command)
        self._action.fill(0.0)
        self._previous_action.fill(0.0)
        self._estimated_velocity.fill(0.0)
        self._episode_length = 0
        self._gait_phase.fill(0.0)
        self._brake_elapsed_s = 0.0
        self._user_active = command_class != "zero"
        self._mode = "MOVE" if self._user_active else "HOLD"
        self._role = "policy"
        np.copyto(self._route_command, command if self._user_active else 0.0)
        single = self._build_single_observation(
            joints.position, joints.velocity, frame.quat_wxyz,
            frame.angular_velocity, self._route_command, self._gait_phase,
        )
        self._history.fill(single)
        self._history.write_into(self._obs)
        np.copyto(self._target, self._parameters.default_position)
        self.publish_output(
            self._target, self._parameters.kp, self._parameters.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def _route(self, frame: InferenceFrame, dt: float, *, advance: bool):
        command = np.asarray(frame.command, dtype=np.float32).reshape(3)
        command_class = classify_commissioning_command(command)
        active = command_class != "zero"
        mode = self._mode
        brake_elapsed = self._brake_elapsed_s
        routed = np.zeros(3, dtype=np.float32)
        if active:
            mode = "MOVE"
            routed[:] = command
            brake_elapsed = 0.0
        else:
            if self._user_active:
                mode, brake_elapsed = "BRAKE", 0.0
            elif mode == "BRAKE" and brake_elapsed >= BRAKE_DURATION_S - 1.0e-9:
                mode = "HOLD"
            if mode == "BRAKE" and advance:
                brake_elapsed += dt
            elif mode != "BRAKE":
                mode = "HOLD"
        if advance:
            self._mode = mode
            self._role = "policy"
            self._user_active = active
            self._brake_elapsed_s = brake_elapsed
            np.copyto(self._route_command, routed)
        return mode, "policy", routed

    def step(self, frame: InferenceFrame, dt: float, *, advance: bool = True) -> PolicyOutput:
        if abs(float(dt) - POLICY_DT_S) > 1.0e-6:
            raise ValueError(f"policy dt must be {POLICY_DT_S:.3f}s, got {dt:.6f}s")
        if not advance:
            np.copyto(self._previous_action_checkpoint, self._previous_action)
        joints = self.bind_joints(frame)
        _mode, _role, routed = self._route(frame, dt, advance=advance)
        if advance:
            episode_length = self._episode_length + 1
            t = episode_length * dt / GAIT_CYCLE_S
            phase = np.mod(t + self._phase_offset, 1.0).astype(np.float32)
        else:
            episode_length = self._episode_length
            phase = self._gait_phase
        single = self._build_single_observation(
            joints.position, joints.velocity, frame.quat_wxyz,
            frame.angular_velocity, routed, phase,
        )
        if advance:
            self._history.append(single)
            self._history.write_into(self._obs)
        else:
            self._history.preview_append_into(single, self._obs)
        value = np.asarray(self._backend.run(self._inputs)["actions"], dtype=np.float32).reshape(-1)
        if value.shape != (ACTION_DIM,):
            raise ValueError(f"policy returned shape {value.shape}")
        if not np.all(np.isfinite(value)):
            raise ValueError("policy returned NaN or Inf")
        np.copyto(self._action, value)
        np.multiply(self._action, self._parameters.action_scale, out=self._scaled_action)
        np.add(self._parameters.default_position, self._scaled_action, out=self._target)
        self._target[WAIST_X_INDEX] = np.clip(
            self._target[WAIST_X_INDEX], -WAIST_X_SAFE_ABS_RAD, WAIST_X_SAFE_ABS_RAD
        )
        if advance:
            self._episode_length = episode_length
            np.copyto(self._gait_phase, phase)
            np.copyto(self._previous_action, self._action)
        else:
            np.copyto(self._previous_action, self._previous_action_checkpoint)
        return self.publish_output(
            self._target, self._parameters.kp, self._parameters.kd,
            estimated_velocity=self._estimated_velocity,
        )

    def commit_preheated_action(self) -> None:
        np.copyto(self._previous_action, self._action)

    def _build_single_observation(self, qj, dqj, quat, omega, command, phase):
        q = np.asarray(qj, dtype=np.float32)
        dq = np.asarray(dqj, dtype=np.float32)
        if q.shape != (ACTION_DIM,) or dq.shape != (ACTION_DIM,):
            raise ValueError("ELF3 policy joint observation must be 29/29")
        single = self._single_obs
        single.fill(0.0)
        single[0:3] = np.asarray(omega, dtype=np.float32)
        single[3:6] = get_gravity_orientation(quat)
        single[6:9] = np.asarray(command, dtype=np.float32)
        np.subtract(q, self._parameters.default_position, out=single[9:38])
        single[38:67] = dq
        single[67:96] = self._previous_action
        phase_value = np.asarray(phase, dtype=np.float32)
        single[96:98] = np.sin(2.0 * np.pi * phase_value)
        single[98:100] = np.cos(2.0 * np.pi * phase_value)
        single[100:102] = self._phase_ratio
        return single

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        backend = getattr(self, "_backend", None)
        if backend is not None:
            backend.close()

    def decode_into(self, outputs) -> None:
        raise NotImplementedError("policy decodes its backend output directly")


__all__ = [
    "ACTION_DIM", "ACTOR_ROLES", "Elf3ThreeActorRouterPolicy", "HISTORY_FRAMES",
    "INPUT_DIM", "POLICY_JOINT_NAMES", "POLICY_DT_S", "SINGLE_FRAME_DIM",
    "BRAKE_DURATION_S", "classify_commissioning_command",
]
