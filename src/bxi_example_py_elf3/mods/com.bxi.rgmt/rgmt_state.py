"""Live HoloRetarget-to-RGMT state for the BXI Mod scheduler."""

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

from .rgmt_policy import RgmtPolicy
from .rgmt_reference import ReferenceReceiver
from .source_service import HoloRetargetSourceService

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


def _release_on_demand(handle: ResourceHandle[object]) -> None:
    """Release resources when the runtime exposes the optional API.

    Older 68 deployments have ``ResourceHandle.get/request`` but no
    ``release`` method.  RGMT must still leave the state cleanly on those
    runtimes instead of failing the control scheduler during a transition.
    The current runtime calls release; older runtimes keep the cached resource
    until controller shutdown.
    """

    release = getattr(handle, "release", None)
    if callable(release):
        release()


class RgmtTeleopState(
    RobotControlState,
    EntryFrameProvider,
    RunningFrameProvider,
):
    """Run the RGMT actor from the existing live HoloRetarget stream."""

    def __init__(
        self,
        name: str,
        state_id: int,
        source: ResourceHandle[HoloRetargetSourceService],
        policy: ResourceHandle[RgmtPolicy],
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
        for parameter, value in (
            ("reference_timeout_s", reference_timeout_s),
            ("max_inter_frame_gap_s", max_inter_frame_gap_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{parameter} must be positive and finite")
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
        self._blend_rgmt_frame: MotorFrame | None = None
        self._blend_output_frame: MotorFrame | None = None
        self._tracking_blend_elapsed_s: float | None = None
        self._tracking_stream_epoch: int | None = None
        self._policy_logger_bound = False
        self._stale_exit_requested = False
        self._prepared = False

    @property
    def policy(self) -> RgmtPolicy:
        return self._policy.get()

    def on_bind(self, ctx: RobotControlContext) -> None:
        self._receiver.configure(
            uri=self.reference_uri,
            max_source_age_s=self.reference_timeout_s,
            max_inter_frame_gap_s=self.max_inter_frame_gap_s,
        )
        self.logger.info(
            "RGMT is disabled while inactive; its CPU ONNX session, "
            "HoloRetarget source and receiver are all on demand"
        )
        self.logger.info(
            f"RGMT reference playout: {self._receiver.playout_description}"
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
                "RGMT HoloRetarget source ready: service="
                f"{source.service_name}, "
                f"started_by_mod={source.started_by_mod}"
            )
            if not self._policy_logger_bound:
                self.policy.bind_logger(self.logger)
                self.logger.info(
                    "RGMT policy ready: providers="
                    f"{self.policy.providers}, warmup="
                    f"{self.policy.cold_warmup_ms:.2f} ms, "
                    "recovery_latch_metadata="
                    f"{self.policy.requires_recovery_latch}"
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
            _release_on_demand(self._policy)
            _release_on_demand(self._source)
            self._policy_logger_bound = False
            raise

    def _release_runtime(self) -> None:
        if not self._prepared:
            return
        self._prepared = False
        self._receiver.stop()
        self._receiver.clear(discontinuity=True)
        _release_on_demand(self._policy)
        _release_on_demand(self._source)
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
        rgmt_natural = self._motor_frame_from_target(ctx, output.joints)
        rgmt = ctx.resolve_motor_frame(
            rgmt_natural,
            self._full_frame_buffer(ctx, "_blend_rgmt_frame"),
        )
        elapsed = min(
            (0.0 if elapsed is None else elapsed) + dt,
            self.tracking_blend_s,
        )
        self._tracking_blend_elapsed_s = elapsed
        alpha = elapsed / self.tracking_blend_s
        blended = self._full_frame_buffer(ctx, "_blend_output_frame")
        for source, target, destination in (
            (normal.qpos, rgmt.qpos, blended.qpos),
            (normal.kp, rgmt.kp, blended.kp),
            (normal.kd, rgmt.kd, blended.kd),
            (normal.vel, rgmt.vel, blended.vel),
            (normal.torque, rgmt.torque, blended.torque),
        ):
            np.subtract(target, source, out=destination)
            destination *= alpha
            destination += source
        return blended

    def on_enter(self, ctx: RobotControlContext) -> None:
        self.logger.info(
            "RGMT 遥操模式已进入：使用过去10 + 当前 + 未来10的21帧窗口；"
            "reference 未就绪时继续运行 Normal 站立策略；第一帧有效输出后"
            f"进行 {self.tracking_blend_s:.3f}s 线性混合"
        )

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        #if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
         #   ctx.request_state(
          #      "com.bxi.basic_actions/zero_torque",
           #     trigger="rgmt_orientation_safety",
            #)
            #return
        # RGMT fall/orientation detection is intentionally disabled.  Keep
        # inference and reference-stream fault handling below unchanged.
        try:
            frame = self.sample_running_frame(ctx, dt, advance=True)
        except Exception as exc:  # noqa: BLE001 - leave tracking on any fault
            self.logger.error(f"RGMT inference failed: {exc}")
            ctx.request_state(
                "com.bxi.basic_actions/normal",
                trigger="rgmt_inference_failure",
            )
            return
        self._apply_frame(ctx, frame)
        if (
            self.policy.should_exit_to_normal
            and not self._stale_exit_requested
        ):
            self._stale_exit_requested = True
            self.logger.warning(
                "RGMT reference 已超时或序列结束；退出到 Normal"
            )
            ctx.request_state(
                "com.bxi.basic_actions/normal",
                trigger="rgmt_reference_lost",
            )


__all__ = ["RgmtTeleopState"]
