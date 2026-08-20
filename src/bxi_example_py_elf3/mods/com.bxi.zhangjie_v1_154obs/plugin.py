from bxi_example_py_elf3.policies import DanceMotionPolicyIsaaclab
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import ZhangjieV1State


POLICY = ResourceKey[DanceMotionPolicyIsaaclab](
    "com.bxi.zhangjie_v1_154obs/policy"
)
END_FRAME = 1690


def _load(context: ResourceLoadContext) -> DanceMotionPolicyIsaaclab:
    policy = DanceMotionPolicyIsaaclab(
        str(context.asset("assets/zhangjie_v1_elf3_0815_2.npz")),
        str(context.asset("assets/zhangjie_v1.onnx")),
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
            "zhangjie_v1_154obs": lambda state: ZhangjieV1State(
                state.name, state.state_id, policy
            )
        }
    )
