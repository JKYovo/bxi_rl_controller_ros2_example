from bxi_example_py_elf3.policies import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .state import SideFlipState


POLICY = ResourceKey[DanceMotionPolicyGravityIsaaclabV3](
    "com.bxi.side_flip/policy"
)


def _load(context: ResourceLoadContext) -> DanceMotionPolicyGravityIsaaclabV3:
    return DanceMotionPolicyGravityIsaaclabV3(
        str(context.asset("assets/side_flip.npz")),
        str(context.asset("assets/side_flip.onnx")),
        start_frame=50,
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load, policy="on_demand")
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "side_flip": lambda state: SideFlipState(
                state.name, state.state_id, policy
            )
        }
    )
