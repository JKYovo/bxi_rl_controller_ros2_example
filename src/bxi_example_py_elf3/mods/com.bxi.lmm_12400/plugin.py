from bxi_example_py_elf3.policies import DanceMotionPolicyIsaaclab
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import Lmm12400State


POLICY = ResourceKey[DanceMotionPolicyIsaaclab](
    "com.bxi.lmm_12400/policy"
)
END_FRAME = 1977


def _load(context: ResourceLoadContext) -> DanceMotionPolicyIsaaclab:
    policy = DanceMotionPolicyIsaaclab(
        str(context.asset("assets/linmeimei_v1_elf3_0814_3.npz")),
        str(context.asset("assets/linmeimei_v1.onnx")),
        start_frame=10,
        fixed_pos=True,
    )
    policy.configure_range(end_frame=END_FRAME)
    return policy


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load, policy="on_demand")
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "lmm_12400": lambda state: Lmm12400State(
                state.name, state.state_id, policy
            )
        }
    )
