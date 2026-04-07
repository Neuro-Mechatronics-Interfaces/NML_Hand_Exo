from __future__ import annotations

from .adapters import NmlHandExoRobotAdapter, VirtualOpenClawRobotAdapter
from .registry import GLOBAL_ROBOT_ADAPTER_REGISTRY, RobotAdapterManifest


def register_builtin_robot_adapters() -> None:
    manifest = RobotAdapterManifest(
        adapter_id="nml_hand_exo",
        display_name="NML Hand Exo",
        description="Adapter for the NML Hand Exo hardware and simulator.",
        tags=("hand", "exo", "rehab", "serial", "tcp"),
    )
    GLOBAL_ROBOT_ADAPTER_REGISTRY.register(manifest, lambda **kwargs: NmlHandExoRobotAdapter(**kwargs))

    openclaw_manifest = RobotAdapterManifest(
        adapter_id="virtual_openclaw",
        display_name="Virtual OpenClaw",
        description="OpenClaw-style virtual robot adapter for portable deployment and testing.",
        tags=("virtual", "openclaw", "gripper", "cross-robot"),
    )
    GLOBAL_ROBOT_ADAPTER_REGISTRY.register(openclaw_manifest, lambda **kwargs: VirtualOpenClawRobotAdapter(**kwargs))
