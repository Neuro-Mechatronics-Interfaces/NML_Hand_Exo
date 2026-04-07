from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROBOT_ADAPTER_CONFIG_VERSION = 1
_ROOT_ALLOWED_KEYS = {"config_version", "adapter_id", "adapter_kwargs", "cli_defaults"}


@dataclass
class RobotAdapterRuntimeConfig:
    adapter_id: str
    adapter_kwargs: dict[str, Any] = field(default_factory=dict)
    cli_defaults: dict[str, Any] = field(default_factory=dict)
    config_version: int = ROBOT_ADAPTER_CONFIG_VERSION


def _default_robot_config_path(adapter_id: str) -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "robot_adapters" / f"{adapter_id}.yaml"


def load_robot_adapter_config(
    adapter_id: str,
    explicit_path: str | None = None,
) -> tuple[RobotAdapterRuntimeConfig, Path | None]:
    path = Path(explicit_path).expanduser().resolve() if explicit_path else _default_robot_config_path(adapter_id)
    if not path.exists():
        return RobotAdapterRuntimeConfig(adapter_id=adapter_id), None

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required to load robot adapter configs. Install pyyaml or remove --robot-config."
        ) from exc

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if payload is None:
        payload = {}
    if not isinstance(payload, dict):
        raise ValueError(f"Robot adapter config must be a mapping: {path}")

    unknown_root_keys = sorted(set(payload.keys()).difference(_ROOT_ALLOWED_KEYS))
    if unknown_root_keys:
        raise ValueError(
            f"Unknown keys in robot adapter config {path}: {', '.join(unknown_root_keys)}"
        )

    raw_config_version = payload.get("config_version", ROBOT_ADAPTER_CONFIG_VERSION)
    try:
        config_version = int(raw_config_version)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"config_version must be an integer in {path}") from exc
    if config_version != ROBOT_ADAPTER_CONFIG_VERSION:
        raise ValueError(
            f"Unsupported config_version {config_version} in {path}. "
            f"Expected {ROBOT_ADAPTER_CONFIG_VERSION}."
        )

    file_adapter = str(payload.get("adapter_id", adapter_id)).strip().lower()
    if file_adapter and file_adapter != adapter_id.strip().lower():
        raise ValueError(
            f"Robot adapter config adapter_id '{file_adapter}' does not match selected adapter '{adapter_id}'."
        )

    adapter_kwargs = payload.get("adapter_kwargs", {})
    cli_defaults = payload.get("cli_defaults", {})
    if not isinstance(adapter_kwargs, dict):
        raise ValueError(f"adapter_kwargs must be a mapping in {path}")
    if not isinstance(cli_defaults, dict):
        raise ValueError(f"cli_defaults must be a mapping in {path}")

    config = RobotAdapterRuntimeConfig(
        adapter_id=adapter_id,
        adapter_kwargs=dict(adapter_kwargs),
        cli_defaults=dict(cli_defaults),
        config_version=config_version,
    )
    return config, path


def apply_cli_defaults_from_config(args, cli_defaults: dict[str, Any], parser_defaults: dict[str, Any]) -> list[str]:
    applied: list[str] = []
    for key, configured_value in cli_defaults.items():
        if key not in parser_defaults:
            continue

        current_value = getattr(args, key, None)
        default_value = parser_defaults[key]
        if current_value == default_value:
            setattr(args, key, configured_value)
            applied.append(key)
    return applied
