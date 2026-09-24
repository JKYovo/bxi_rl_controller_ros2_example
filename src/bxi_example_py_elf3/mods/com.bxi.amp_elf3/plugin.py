from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .policy import Elf3AmpPolicy
from .state import Elf3AmpState


AMP_POLICY = ResourceKey[Elf3AmpPolicy]("com.bxi.amp_elf3/policy")


def _load_policy(context: ResourceLoadContext) -> Elf3AmpPolicy:
    #return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Rough-V4_model_10300.onnx")))
    #return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Flat-V4-Loco_model_1100.onnx")))
    #return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Rough-V4_model_104500.onnx")))
    #return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Flat-V3_model_47400.onnx"))) #从头训的v3
    #return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Rough-V4_model_22800.onnx")))
    #return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Flat-V4_model_34000.onnx")))#天工地形+不起身
    return Elf3AmpPolicy(str(context.asset("assets/BXI-ELF3-AMP-Rough-V4-Delay_model_41000.onnx")))#天工地形+起身

def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(AMP_POLICY, _load_policy, policy="on_demand")
    policy = context.resource(AMP_POLICY)
    return ModDefinition(
        state_factories={
            "amp_elf3": lambda state: Elf3AmpState(
                state.name, state.state_id, policy
            )
        }
    )
