from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from bxi_example_py_elf3.policies import HumanoidGaitPolicyLiteIsaaclab

from .rgmt_policy import RgmtPolicy
from .rgmt_reference import ReferenceReceiver
from .rgmt_state import RgmtTeleopState
from .source_service import HoloRetargetSourceService

POLICY = ResourceKey[RgmtPolicy]("com.bxi.rgmt/policy")
SOURCE = ResourceKey[HoloRetargetSourceService]("com.bxi.rgmt/source")
NORMAL_POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab](
    "com.bxi.basic_actions/normal_policy"
)


def create_mod(context: ModLoadContext) -> ModDefinition:
    receiver = ReferenceReceiver()

    def load_source(
        _resource: ResourceLoadContext,
    ) -> HoloRetargetSourceService:
        return HoloRetargetSourceService()

    def load_policy(resource: ResourceLoadContext) -> RgmtPolicy:
        return RgmtPolicy(
            model_path=resource.asset("assets/rgmtr_40200.onnx"),
            receiver=receiver,
            reference_yaw_mode="initial",
        )

    # RGMT owns separate on-demand resources and does not import, request or
    # retain resources from another teleoperation Mod.
    context.register_resource(SOURCE, load_source, policy="on_demand")
    context.register_resource(POLICY, load_policy, policy="on_demand")
    source = context.resource(SOURCE)
    policy = context.resource(POLICY)
    normal_policy = context.resource(NORMAL_POLICY)
    return ModDefinition(
        state_factories={
            "rgmt_teleop": lambda state: RgmtTeleopState(
                state.name,
                state.state_id,
                source,
                policy,
                normal_policy,
                receiver,
                reference_uri=state.string_param(
                    "reference_uri", "tcp://127.0.0.1:6001"
                ),
                reference_timeout_s=state.float_param(
                    "reference_timeout_s", 0.6
                ),
                max_inter_frame_gap_s=state.float_param(
                    "max_inter_frame_gap_s", 0.06
                ),
                tracking_blend_s=state.float_param("tracking_blend_s", 0.4),
            )
        }
    )


__all__ = ["NORMAL_POLICY", "POLICY", "SOURCE", "create_mod"]
