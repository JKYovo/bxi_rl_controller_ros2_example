from bxi_example_py_elf3.policies import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import ZjNewEasyV3State


POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3](
    "com.bxi.zj_new_easy_v3/policy"
)
END_FRAME = 1920


def _load(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    policy = DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/zj_new_easy.npz")),
        str(context.asset("assets/zj_new_easy.onnx")),
        start_frame=60,
        fixed_pos=True,
    )
    policy.configure_range(end_frame=END_FRAME)
    return policy


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load, policy="on_demand")
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "zj_new_easy_v3": lambda state: ZjNewEasyV3State(
                state.name, state.state_id, policy
            )
        }
    )
