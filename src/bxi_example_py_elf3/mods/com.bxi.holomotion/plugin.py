from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from bxi_example_py_elf3.policies import HumanoidGaitPolicyLiteIsaaclab

from .policy import HoloMotionPolicy
from .reference import ReferenceReceiver
from .source_service import HoloRetargetSourceService
from .state import HoloMotionTeleopState

POLICY = ResourceKey[HoloMotionPolicy]("com.bxi.holomotion/policy")
SOURCE = ResourceKey[HoloRetargetSourceService]("com.bxi.holomotion/source")
NORMAL_POLICY = ResourceKey[HumanoidGaitPolicyLiteIsaaclab](
    "com.bxi.basic_actions/normal_policy"
)


def create_mod(context: ModLoadContext) -> ModDefinition:
    receiver = ReferenceReceiver()

    def load_source(
        _resource: ResourceLoadContext,
    ) -> HoloRetargetSourceService:
        return HoloRetargetSourceService()

    def load_policy(resource: ResourceLoadContext) -> HoloMotionPolicy:
        return HoloMotionPolicy(
            model_path=resource.asset(
                "assets/native_affine_migrated_1000_20260901/model_25000.onnx"
            ),
            receiver=receiver,
        )

    # Keep the 1.6 GB ONNX session out of the shared controller until the
    # operator explicitly requests HoloMotion. ResourceManager creates the
    # isolated inference process off the control thread, and the state ends it
    # again on exit.
    context.register_resource(SOURCE, load_source, policy="on_demand")
    context.register_resource(POLICY, load_policy, policy="on_demand")
    source = context.resource(SOURCE)
    policy = context.resource(POLICY)
    normal_policy = context.resource(NORMAL_POLICY)
    return ModDefinition(
        state_factories={
            "holomotion_teleop": lambda state: HoloMotionTeleopState(
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
            ),
        }
    )


__all__ = ["NORMAL_POLICY", "POLICY", "SOURCE", "create_mod"]
