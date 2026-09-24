from __future__ import annotations

import hashlib
import json
from pathlib import Path

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .router_policy import Elf3ThreeActorRouterPolicy
from .safe_walk_state import SafeThreeActorWalkState


POLICY = ResourceKey[Elf3ThreeActorRouterPolicy](
    "com.local.elf3_three_actor_router/policy"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_policy(context: ResourceLoadContext) -> Elf3ThreeActorRouterPolicy:
    manifest_path = Path(context.asset("assets/policy_manifest.json"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "UNQUALIFIED_OFFLINE_ONLY":
        raise ValueError("policy manifest is not the expected offline artifact")
    model = Path(context.asset(manifest["onnx_file"]))
    actual = _sha256(model)
    if actual != manifest["onnx_sha256"]:
        raise ValueError(
            f"ELF3 policy checksum mismatch: expected {manifest['onnx_sha256']}, "
            f"got {actual}"
        )
    return Elf3ThreeActorRouterPolicy(model)


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(POLICY, _load_policy, policy="on_demand")
    policy = context.resource(POLICY)
    return ModDefinition(
        state_factories={
            "safe_three_actor_walk": lambda state: SafeThreeActorWalkState(
                state.name,
                state.state_id,
                policy,
            )
        }
    )


__all__ = ["create_mod"]
