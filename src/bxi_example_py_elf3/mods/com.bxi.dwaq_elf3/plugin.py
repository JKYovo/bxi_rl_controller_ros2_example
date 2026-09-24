from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .policy import Elf3DwaqPolicy
from .state import Elf3DwaqState


DWAQ_POLICY = ResourceKey[Elf3DwaqPolicy]("com.bxi.dwaq_elf3/policy")


def _load_policy(resource: ResourceLoadContext) -> Elf3DwaqPolicy:
    return Elf3DwaqPolicy(
        resource.asset("assets/policy_281900_idle.onnx"),
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(DWAQ_POLICY, _load_policy, policy="on_demand")
    policy = context.resource(DWAQ_POLICY)
    return ModDefinition(
        state_factories={
            "dwaq_elf3": lambda state: Elf3DwaqState(
                state.name,
                state.state_id,
                policy,
            )
        }
    )


__all__ = ["DWAQ_POLICY", "create_mod"]
