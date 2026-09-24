from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from bxi_example_py_elf3.framework.mod_api import (
    ResourceHandle,
    RobotControlState,
    StateBehavior,
)
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

from .policy import Elf3AmpPolicy
from .policy import ELF3_AMP_COMMAND_MAX, ELF3_AMP_COMMAND_MIN

_TRANSLATION_THRESHOLD = np.float32(0.05)
_TURNING_MAX_ABS_ANG_VEL = np.float32(2.0)
_TRANSLATING_MAX_ABS_ANG_VEL = np.float32(1.0)

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class Elf3AmpState(
    RobotControlState, EntryFrameProvider, RunningFrameProvider
):
    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[Elf3AmpPolicy],
    ) -> None:
        super().__init__(name, state_id, resources=(policy,))
        self._policy = policy
        self._previous_command = np.zeros(3, dtype=np.float32)
        self._command = np.zeros(3, dtype=np.float32)

    def get_cmd_vel(self, ctx: RobotControlContext) -> np.ndarray:
        # Use the command already received and owned by the controller runtime.
        # A second ROS subscription plus publisher-count gating can spuriously
        # invalidate every axis on a real system with multiple endpoints.
        command = self._profile_cmd_vel(ctx)
        return self._publish_cmd_vel(ctx, self.process_cmd_vel(ctx, command))

    @property
    def policy(self) -> Elf3AmpPolicy:
        return self._policy.get()

    def on_prepare(
        self,
        ctx: RobotControlContext,
        from_state: StateBehavior[RobotControlContext],
    ) -> None:
        ctx.preheat_model(self.policy, command=self.get_cmd_vel(ctx))

    def on_enter(self, ctx: RobotControlContext) -> None:
        self._previous_command.fill(0.0)
        self._command.fill(0.0)

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._motor_frame_from_target(ctx, self.policy.output.joints)

    def process_cmd_vel(
        self, ctx: RobotControlContext, cmd_vel: np.ndarray
    ) -> np.ndarray:
        # Preserve the existing translation filter; commands remain physical
        # velocities inside V4's envelope, without observation normalization.
        bounded_cmd = np.clip(
            np.asarray(cmd_vel, dtype=np.float32).reshape(3),
            ELF3_AMP_COMMAND_MIN,
            ELF3_AMP_COMMAND_MAX,
        )
        self._command[:2] = (
            0.98 * self._previous_command[:2] + 0.02 * bounded_cmd[:2]
        )
        translating = bool(
            np.any(np.abs(bounded_cmd[:2]) > _TRANSLATION_THRESHOLD)
            or np.any(np.abs(self._command[:2]) > _TRANSLATION_THRESHOLD)
        )
        yaw_limit = (
            _TRANSLATING_MAX_ABS_ANG_VEL
            if translating
            else _TURNING_MAX_ABS_ANG_VEL
        )
        self._command[2] = np.clip(bounded_cmd[2], -yaw_limit, yaw_limit)
        self._previous_command[:] = self._command
        return self._command

    def sample_running_frame(
        self,
        ctx: RobotControlContext,
        dt: float,
        *,
        advance: bool,
    ) -> MotorFrame:
        self.get_cmd_vel(ctx)
        output = self.policy.step(ctx.inference_frame, dt, advance=advance)
        return self._motor_frame_from_target(ctx, output.joints)

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        # Deliberately keep the AMP policy running after a large pitch/roll.
        # This Mod is used to inspect recovery/fall behavior in simulation;
        # manual zero-torque remains available through the normal remote event.
        self._apply_frame(
            ctx, self.sample_running_frame(ctx, dt, advance=True)
        )
