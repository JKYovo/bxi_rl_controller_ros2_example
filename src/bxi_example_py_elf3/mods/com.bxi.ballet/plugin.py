from bxi_example_py_elf3.policies import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)
from .state import BalletState


POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3]("com.bxi.ballet/policy")
END_FRAME = 1680


def _load(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    policy = DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/zj.npz")),
        str(context.asset("assets/zj_4w.onnx")),
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
            "ballet": lambda state: BalletState(state.name, state.state_id, policy)
        }
    )
