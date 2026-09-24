from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bxi_example_py_elf3.framework.inference.contract import JointInputBinding
from bxi_example_py_elf3.framework.mod_api import ResourceHandle, RobotControlState
from bxi_example_py_elf3.framework.mod_api import StateBehavior
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

from .router_policy import Elf3ThreeActorRouterPolicy

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


_HARD_LOWER = np.asarray(
    (
        -2.8798, -2.8798, -0.5236, -0.34907, -3.0543, -0.2618,
        -2.8798, -2.8798, -2.8798, -0.95993, -0.95993, -2.8798,
        -2.8798, -2.8798, -2.8798, -0.48869, -3.0543, -1.309,
        -1.309, -2.8798, -2.8798, -0.7854, -0.7854, -0.087266,
        -0.087266, -0.87266, -0.87266, -0.34907, -0.34907,
    ),
    dtype=np.float32,
)
_HARD_UPPER = np.asarray(
    (
        2.8798, 2.8798, 0.5236, 3.0543, 0.34907, 0.2618,
        2.8798, 2.8798, 2.8798, 1.6581, 1.6581, 2.8798,
        2.8798, 2.8798, 2.8798, 3.0543, 0.48869, 1.309,
        1.309, 2.8798, 2.8798, 0.7854, 0.7854, 2.618,
        2.618, 0.7854, 0.7854, 0.34907, 0.34907,
    ),
    dtype=np.float32,
)
_COMMAND_EPSILON = 1.0e-4
_CANONICAL_COMMAND_POSITIVE = np.asarray((0.375, 0.20, 0.40), dtype=np.float32)
_CANONICAL_COMMAND_NEGATIVE = np.asarray((-0.25, -0.20, -0.40), dtype=np.float32)
_SIMULATION_TOPIC_PREFIX = "simulation/"


class SafeThreeActorWalkState(
    RobotControlState,
    EntryFrameProvider,
    RunningFrameProvider,
):
    """Fail-closed吊架 commissioning state for the TienKung AMP policy."""

    _SOFT_LIMIT_FACTOR = 0.90
    _MAX_SENSOR_AGE_NS = 60_000_000
    _MAX_RAW_ACTION_ABS = 8.0
    _MAX_TARGET_DELTA_PER_CYCLE = 0.12
    _MIN_QUATERNION_NORM = 0.90
    _MAX_QUATERNION_NORM = 1.10
    _MAX_ABS_JOINT_VELOCITY = 30.0
    _MAX_ABS_ANGULAR_VELOCITY = 20.0
    _JOINT_POSITION_LIMIT_TOLERANCE = 0.02

    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[Elf3ThreeActorRouterPolicy],
    ) -> None:
        super().__init__(name, state_id, resources=(policy,))
        self._policy = policy
        midpoint = 0.5 * (_HARD_LOWER + _HARD_UPPER)
        half_range = 0.5 * (_HARD_UPPER - _HARD_LOWER)
        self._soft_lower = midpoint - self._SOFT_LIMIT_FACTOR * half_range
        self._soft_upper = midpoint + self._SOFT_LIMIT_FACTOR * half_range
        self._safe_target = np.zeros(29, dtype=np.float32)
        self._last_target = np.zeros(29, dtype=np.float32)
        self._last_finite_failsafe_target = np.zeros(29, dtype=np.float32)
        self._zero_gain = np.zeros(29, dtype=np.float32)
        self._safety_joint_binding = JointInputBinding(
            Elf3ThreeActorRouterPolicy.joint_contract
        )
        self._fault_latched = False
        self._first_running_frame_pending = False

    @property
    def policy(self) -> Elf3ThreeActorRouterPolicy:
        return self._policy.get()

    def is_available(self, ctx: RobotControlContext) -> bool:
        safe, reason = self._inputs_are_safe(ctx)
        if not safe:
            self.logger.warning(f"ELF3 AMP entry rejected: {reason}")
        return safe

    def on_prepare(
        self,
        ctx: RobotControlContext,
        from_state: StateBehavior[RobotControlContext],
    ) -> None:
        safe, reason = self._inputs_are_safe(ctx)
        if not safe:
            raise ValueError(f"ELF3 AMP preparation rejected: {reason}")
        ctx.preheat_model(self.policy, command=self.get_cmd_vel(ctx))

    def on_enter(self, ctx: RobotControlContext) -> None:
        self._fault_latched = False
        joints = self.policy.bind_joints(ctx.inference_frame)
        if np.all(np.isfinite(joints.position)):
            np.clip(joints.position, self._soft_lower, self._soft_upper, out=self._last_target)
        else:
            self._last_target.fill(0.0)
        np.copyto(self._last_finite_failsafe_target, self._last_target)
        self.policy.commit_preheated_action()
        self._first_running_frame_pending = True

    def process_cmd_vel(
        self,
        ctx: RobotControlContext,
        cmd_vel: np.ndarray,
    ) -> np.ndarray:
        """Map normal-mode directional input onto the frozen command card."""

        del ctx
        for axis in range(3):
            if cmd_vel[axis] > _COMMAND_EPSILON:
                cmd_vel[axis] = _CANONICAL_COMMAND_POSITIVE[axis]
            elif cmd_vel[axis] < -_COMMAND_EPSILON:
                cmd_vel[axis] = _CANONICAL_COMMAND_NEGATIVE[axis]
            else:
                cmd_vel[axis] = 0.0
        return cmd_vel

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        safe, reason = self._inputs_are_safe(ctx)
        if not safe:
            return self._entry_fail_closed_frame(ctx, reason)
        return self._checked_policy_frame(ctx, advance_target=False)

    def _entry_fail_closed_frame(
        self,
        ctx: RobotControlContext,
        reason: str,
    ) -> MotorFrame:
        self.logger.error(f"ELF3 AMP entry frame rejected: {reason}")
        joints = self.policy.bind_joints(ctx.inference_frame)
        ctx.request_state(
            "com.bxi.basic_actions/zero_torque",
            trigger="elf3_three_actor_entry_safety",
            force=True,
        )
        return self._motor_frame(
            ctx,
            (
                np.clip(joints.position, self._soft_lower, self._soft_upper)
                if np.all(np.isfinite(joints.position))
                else self._last_finite_failsafe_target
            ),
            self._zero_gain,
            self._zero_gain,
            layout=joints.layout,
        )

    def _inputs_are_safe(self, ctx: RobotControlContext) -> tuple[bool, str]:
        topic_prefix = getattr(ctx.ros_node, "topic_prefix", None)
        if topic_prefix != _SIMULATION_TOPIC_PREFIX:
            return (
                False,
                "UNQUALIFIED policy is simulation-only; "
                f"topic_prefix={topic_prefix!r}",
            )
        now_ns = int(ctx.ros_node.get_clock().now().nanoseconds)
        joint_timestamp_ns = int(ctx.robot_joints.timestamp_ns)
        joint_age_ns = now_ns - joint_timestamp_ns
        if (
            joint_timestamp_ns <= 0
            or joint_age_ns < 0
            or joint_age_ns > self._MAX_SENSOR_AGE_NS
        ):
            return False, f"joint state age is unsafe: {joint_age_ns / 1.0e6:.2f} ms"
        for name, value in (
            ("joint position", ctx.robot_joints.position),
            ("joint velocity", ctx.robot_joints.velocity),
            ("orientation", ctx.current_quat_xyzw),
            ("angular velocity", ctx.current_omega),
            ("raw command", ctx.current_raw_cmd_vel),
        ):
            if not np.all(np.isfinite(value)):
                return False, f"{name} contains NaN or Inf"
        quaternion_norm = float(np.linalg.norm(ctx.current_quat_xyzw))
        if not self._MIN_QUATERNION_NORM <= quaternion_norm <= self._MAX_QUATERNION_NORM:
            return False, f"orientation quaternion norm is unsafe: {quaternion_norm:.6f}"
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            return False, "roll or pitch exceeds 60 degrees"
        try:
            joints = self._safety_joint_binding.bind(ctx.inference_frame.joints)
        except (KeyError, TypeError, ValueError) as exc:
            return False, f"joint layout is incompatible: {exc}"
        if np.any(
            joints.position < (_HARD_LOWER - self._JOINT_POSITION_LIMIT_TOLERANCE)
        ) or np.any(
            joints.position > (_HARD_UPPER + self._JOINT_POSITION_LIMIT_TOLERANCE)
        ):
            return False, "joint position exceeds URDF hard-limit envelope"
        joint_velocity_abs = float(np.max(np.abs(joints.velocity), initial=0.0))
        if joint_velocity_abs > self._MAX_ABS_JOINT_VELOCITY:
            return False, f"joint velocity is unsafe: {joint_velocity_abs:.3f} rad/s"
        angular_velocity_abs = float(
            np.max(np.abs(ctx.current_omega), initial=0.0)
        )
        if angular_velocity_abs > self._MAX_ABS_ANGULAR_VELOCITY:
            return False, f"angular velocity is unsafe: {angular_velocity_abs:.3f} rad/s"
        return True, ""

    def _checked_policy_frame(
        self,
        ctx: RobotControlContext,
        *,
        advance_target: bool,
    ) -> MotorFrame:
        raw_action = np.asarray(self.policy._action)
        target = self.policy.output.joints
        if not np.all(np.isfinite(raw_action)):
            raise ValueError("actor output contains NaN or Inf")
        raw_abs = float(np.max(np.abs(raw_action), initial=0.0))
        if raw_abs > self._MAX_RAW_ACTION_ABS:
            raw_index = int(np.argmax(np.abs(raw_action)))
            joints = self.policy.bind_joints(ctx.inference_frame)
            observation = np.asarray(self.policy._obs).reshape(-1)
            raise ValueError(
                f"actor output magnitude {raw_abs:.3f} exceeds "
                f"{self._MAX_RAW_ACTION_ABS:.3f}; role={self.policy.role}, "
                f"joint={joints.layout.names[raw_index]}, index={raw_index}, "
                f"value={float(raw_action[raw_index]):.6f}, "
                f"episode={self.policy._episode_length}, "
                f"observation_abs_max="
                f"{float(np.max(np.abs(observation), initial=0.0)):.6f}, "
                f"joint_position_abs_max="
                f"{float(np.max(np.abs(joints.position), initial=0.0)):.6f}, "
                f"joint_velocity_abs_max="
                f"{float(np.max(np.abs(joints.velocity), initial=0.0)):.6f}"
            )
        if not all(
            np.all(np.isfinite(value))
            for value in (target.position, target.kp, target.kd)
        ):
            raise ValueError("policy target contains NaN or Inf")
        np.clip(target.position, self._soft_lower, self._soft_upper, out=self._safe_target)
        np.clip(
            self._safe_target,
            self._last_target - self._MAX_TARGET_DELTA_PER_CYCLE,
            self._last_target + self._MAX_TARGET_DELTA_PER_CYCLE,
            out=self._safe_target,
        )
        if advance_target:
            np.copyto(self._last_target, self._safe_target)
            np.copyto(self._last_finite_failsafe_target, self._safe_target)
        return self._motor_frame(
            ctx,
            self._safe_target,
            target.kp,
            target.kd,
            layout=target.layout,
        )

    def sample_running_frame(
        self,
        ctx: RobotControlContext,
        dt: float,
        *,
        advance: bool,
    ) -> MotorFrame:
        command = self.get_cmd_vel(ctx)
        if not np.all(np.isfinite(command)):
            raise ValueError("command contains NaN or Inf")
        if self._first_running_frame_pending:
            if advance:
                self._first_running_frame_pending = False
        else:
            self.policy.step(ctx.inference_frame, dt, advance=advance)
        return self._checked_policy_frame(ctx, advance_target=advance)

    def _fail_closed(self, ctx: RobotControlContext, reason: str) -> None:
        if not self._fault_latched:
            self.logger.error(f"ELF3 AMP safety gate tripped: {reason}")
            self._fault_latched = True
        zero_frame = self._motor_frame(
            ctx,
            self._last_finite_failsafe_target,
            self._zero_gain,
            self._zero_gain,
            layout=self.policy.output.joints.layout,
        )
        self._apply_frame(ctx, zero_frame)
        ctx.request_state(
            "com.bxi.basic_actions/zero_torque",
            trigger="elf3_three_actor_safety",
            force=True,
        )

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        safe, reason = self._inputs_are_safe(ctx)
        if not safe:
            self._fail_closed(ctx, reason)
            return
        try:
            frame = self.sample_running_frame(ctx, dt, advance=True)
        except Exception as exc:
            self._fail_closed(ctx, f"{type(exc).__name__}: {exc}")
            return
        self._apply_frame(ctx, frame)


__all__ = ["SafeThreeActorWalkState"]
