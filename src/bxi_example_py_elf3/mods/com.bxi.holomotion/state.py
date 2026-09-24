from __future__ import annotations

import math
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
from bxi_example_py_elf3.policies import HumanoidGaitPolicyLiteIsaaclab

from .policy import HoloMotionPolicy
from .reference import ReferenceReceiver
from .source_service import HoloRetargetSourceService

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class HoloMotionTeleopState(
    RobotControlState,
    EntryFrameProvider,
    RunningFrameProvider,
):
    """Run HoloMotion inside the BXI control scheduler."""

    def __init__(
        self,
        name: str,
        state_id: int,
        source: ResourceHandle[HoloRetargetSourceService],
        policy: ResourceHandle[HoloMotionPolicy],
        normal_policy: ResourceHandle[HumanoidGaitPolicyLiteIsaaclab],
        receiver: ReferenceReceiver,
        *,
        reference_uri: str,
        reference_timeout_s: float,
        max_inter_frame_gap_s: float,
        tracking_blend_s: float,
    ) -> None:
        super().__init__(
            name,
            state_id,
            resources=(source, policy, normal_policy),
        )
        if not reference_uri:
            raise ValueError("reference_uri must not be empty")
        if (
            not math.isfinite(reference_timeout_s)
            or reference_timeout_s <= 0.0
        ):
            raise ValueError("reference_timeout_s must be positive and finite")
        if (
            not math.isfinite(max_inter_frame_gap_s)
            or max_inter_frame_gap_s <= 0.0
        ):
            raise ValueError(
                "max_inter_frame_gap_s must be positive and finite"
            )
        if not math.isfinite(tracking_blend_s) or tracking_blend_s < 0.0:
            raise ValueError(
                "tracking_blend_s must be finite and non-negative"
            )
        self._source = source
        self._policy = policy
        self._normal_policy = normal_policy
        self._receiver = receiver
        self.reference_uri = reference_uri
        self.reference_timeout_s = float(reference_timeout_s)
        self.max_inter_frame_gap_s = float(max_inter_frame_gap_s)
        self.tracking_blend_s = float(tracking_blend_s)
        self._last_running_frame: MotorFrame | None = None
        self._blend_normal_frame: MotorFrame | None = None
        self._blend_policy_frame: MotorFrame | None = None
        self._blend_output_frame: MotorFrame | None = None
        self._tracking_blend_elapsed_s: float | None = None
        self._tracking_stream_epoch: int | None = None
        self._policy_logger_bound = False
        self._stale_exit_requested = False
        self._prepared = False

    @property
    def policy(self) -> HoloMotionPolicy:
        return self._policy.get()

    def on_bind(self, ctx: RobotControlContext) -> None:
        self._receiver.configure(
            uri=self.reference_uri,
            max_source_age_s=self.reference_timeout_s,
            max_inter_frame_gap_s=self.max_inter_frame_gap_s,
        )
        self.logger.info(
            "HoloMotion is disabled while inactive; HoloRetarget, policy and "
            "reference receiver start only after an explicit transition request"
        )
        self.logger.info(
            "HoloMotion reference playout: "
            f"{self._receiver.playout_description}"
        )

    def on_unbind(self, ctx: RobotControlContext) -> None:
        self._receiver.stop()

    def on_prepare(
        self,
        ctx: RobotControlContext,
        from_state: StateBehavior[RobotControlContext],
    ) -> None:
        try:
            self._receiver.start()
            source = self._source.get()
            self.logger.info(
                "HoloRetarget source ready: service="
                f"{source.service_name}, started_by_mod={source.started_by_mod}"
            )
            if not self._policy_logger_bound:
                self.policy.bind_logger(self.logger)
                self.logger.info(
                    "HoloMotion policy ready: providers="
                    f"{self.policy.providers}, on_demand_warmup="
                    f"{self.policy.cold_warmup_ms:.2f} ms"
                )
                self._policy_logger_bound = True
            self._last_running_frame = None
            self._tracking_blend_elapsed_s = None
            self._tracking_stream_epoch = None
            self._stale_exit_requested = False
            self.policy.reset(ctx.inference_frame)
            self._prepared = True
        except BaseException:
            self._receiver.stop()
            self._policy.release()
            self._source.release()
            self._policy_logger_bound = False
            raise

    def _release_runtime(self) -> None:
        if not self._prepared:
            return
        self._prepared = False
        self._receiver.stop()
        self._receiver.clear(discontinuity=True)
        self._policy.release()
        self._source.release()
        self._policy_logger_bound = False
        self._last_running_frame = None
        self._tracking_blend_elapsed_s = None
        self._tracking_stream_epoch = None

    def on_prepare_cancel(
        self,
        ctx: RobotControlContext,
        from_state: StateBehavior[RobotControlContext],
    ) -> None:
        self._release_runtime()

    def on_exit(self, ctx: RobotControlContext) -> None:
        self._release_runtime()

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._motor_frame_from_target(
            ctx,
            self._normal_policy.get().output.joints,
        )

    def _sample_normal_waiting_frame(
        self,
        ctx: RobotControlContext,
        dt: float,
        *,
        advance: bool,
    ) -> MotorFrame:
        self.get_cmd_vel(ctx)
        output = self._normal_policy.get().step(
            ctx.inference_frame,
            dt,
            advance=advance,
        )
        return self._motor_frame_from_target(ctx, output.joints)

    def sample_running_frame(
        self,
        ctx: RobotControlContext,
        dt: float,
        *,
        advance: bool,
    ) -> MotorFrame:
        if not advance:
            return self._last_running_frame or self.get_entry_frame(ctx)
        output = self.policy.step(ctx.inference_frame, dt, advance=True)
        if not self.policy.started:
            frame = self._sample_normal_waiting_frame(
                ctx,
                dt,
                advance=True,
            )
        else:
            if self.policy.stream_epoch != self._tracking_stream_epoch:
                self._tracking_stream_epoch = self.policy.stream_epoch
                self._tracking_blend_elapsed_s = None
            frame = self._sample_tracking_frame(ctx, output, dt)
        self._last_running_frame = frame
        return frame

    def _full_frame_buffer(
        self,
        ctx: RobotControlContext,
        attribute: str,
    ) -> MotorFrame:
        frame = getattr(self, attribute)
        if frame is None or frame.layout != ctx.robot_layout:
            frame = MotorFrame.empty(ctx.robot_layout)
            setattr(self, attribute, frame)
        return frame

    def _sample_tracking_frame(
        self,
        ctx: RobotControlContext,
        output,
        dt: float,
    ) -> MotorFrame:
        elapsed = self._tracking_blend_elapsed_s
        if self.tracking_blend_s == 0.0 or (
            elapsed is not None and elapsed >= self.tracking_blend_s
        ):
            self._tracking_blend_elapsed_s = self.tracking_blend_s
            return self._motor_frame_from_target(ctx, output.joints)

        normal_natural = self._sample_normal_waiting_frame(
            ctx,
            dt,
            advance=True,
        )
        normal = ctx.resolve_motor_frame(
            normal_natural,
            self._full_frame_buffer(ctx, "_blend_normal_frame"),
        )
        policy_natural = self._motor_frame_from_target(ctx, output.joints)
        policy = ctx.resolve_motor_frame(
            policy_natural,
            self._full_frame_buffer(ctx, "_blend_policy_frame"),
        )
        elapsed = min(
            (0.0 if elapsed is None else elapsed) + dt,
            self.tracking_blend_s,
        )
        self._tracking_blend_elapsed_s = elapsed
        alpha = elapsed / self.tracking_blend_s
        blended = self._full_frame_buffer(ctx, "_blend_output_frame")
        for source, target, destination in (
            (normal.qpos, policy.qpos, blended.qpos),
            (normal.kp, policy.kp, blended.kp),
            (normal.kd, policy.kd, blended.kd),
            (normal.vel, policy.vel, blended.vel),
            (normal.torque, policy.torque, blended.torque),
        ):
            np.subtract(target, source, out=destination)
            destination *= alpha
            destination += source
        return blended

    def on_enter(self, ctx: RobotControlContext) -> None:
        self.logger.info(
            "HoloMotion 模式已进入："
            "等待连续 11 帧 reference（current + 10 future）；"
            "等待期间保持 Normal 站立输出；第一帧有效输出后"
            f"进行 {self.tracking_blend_s:.3f}s 线性混合"
        )

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            ctx.request_state(
                "com.bxi.basic_actions/zero_torque",
                trigger="holomotion_orientation_safety",
            )
            return
        try:
            frame = self.sample_running_frame(ctx, dt, advance=True)
        except Exception as exc:  # noqa: BLE001 - return safely from any policy fault
            self.logger.error(f"HoloMotion inference failed: {exc}")
            ctx.request_state(
                "com.bxi.basic_actions/normal",
                trigger="holomotion_inference_failure",
            )
            return
        self._apply_frame(ctx, frame)
        if (
            self.policy.should_exit_to_normal
            and not self._stale_exit_requested
        ):
            self._stale_exit_requested = True
            self.logger.warning(
                "HoloMotion reference 已超时或序列结束；按官方遥操语义退出到 Normal"
            )
            ctx.request_state(
                "com.bxi.basic_actions/normal",
                trigger="holomotion_reference_lost",
            )


__all__ = ["HoloMotionTeleopState"]
