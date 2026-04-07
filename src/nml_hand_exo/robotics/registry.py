from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from .contracts import RobotAdapter


RobotAdapterFactory = Callable[..., RobotAdapter]


@dataclass(frozen=True)
class RobotAdapterManifest:
    adapter_id: str
    display_name: str
    description: str
    tags: tuple[str, ...] = tuple()
    defaults: dict[str, Any] = field(default_factory=dict)


@dataclass
class _RegisteredAdapter:
    manifest: RobotAdapterManifest
    factory: RobotAdapterFactory


class RobotAdapterRegistry:
    def __init__(self) -> None:
        self._registry: dict[str, _RegisteredAdapter] = {}

    def register(self, manifest: RobotAdapterManifest, factory: RobotAdapterFactory) -> None:
        adapter_id = manifest.adapter_id.strip().lower()
        if not adapter_id:
            raise ValueError("adapter_id must be non-empty")
        self._registry[adapter_id] = _RegisteredAdapter(manifest=manifest, factory=factory)

    def create(self, adapter_id: str, **overrides: Any) -> RobotAdapter:
        key = adapter_id.strip().lower()
        if key not in self._registry:
            known = ", ".join(sorted(self._registry)) or "<none>"
            raise KeyError(f"Unknown robot adapter '{adapter_id}'. Available adapters: {known}")

        registered = self._registry[key]
        kwargs = dict(registered.manifest.defaults)
        kwargs.update(overrides)
        adapter = registered.factory(**kwargs)
        adapter.initialize()

        health = adapter.health_check()
        if not health.healthy:
            adapter.shutdown()
            raise RuntimeError(
                f"Adapter '{registered.manifest.adapter_id}' failed health check: {health.message}"
            )

        report = adapter.compatibility_report()
        if not report.compatible:
            adapter.shutdown()
            raise RuntimeError(
                f"Adapter '{registered.manifest.adapter_id}' failed compatibility check: {report.message}"
            )
        return adapter

    def list_manifests(self) -> list[RobotAdapterManifest]:
        return [self._registry[key].manifest for key in sorted(self._registry)]

    def has(self, adapter_id: str) -> bool:
        return adapter_id.strip().lower() in self._registry


GLOBAL_ROBOT_ADAPTER_REGISTRY = RobotAdapterRegistry()
