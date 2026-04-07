from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


POLICY_PROFILE_VERSION = 1
_POLICY_PROFILE_ALLOWED_KEYS = {"profile_version", "profile_id", "description", "defaults"}


@dataclass
class PolicyProfileManifest:
    profile_id: str
    description: str = ""
    defaults: dict[str, Any] = field(default_factory=dict)
    profile_version: int = POLICY_PROFILE_VERSION


def _default_policy_profile_path(profile_id: str) -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "policy_profiles" / f"{profile_id}.yaml"


def list_available_policy_profiles() -> list[str]:
    root = Path(__file__).resolve().parents[3] / "config" / "policy_profiles"
    if not root.exists():
        return []
    return sorted(path.stem for path in root.glob("*.yaml"))


def load_policy_profile_manifest(
    profile_id: str = "",
    explicit_path: str | None = None,
) -> tuple[PolicyProfileManifest, Path]:
    if explicit_path:
        path = Path(explicit_path).expanduser().resolve()
    else:
        if not profile_id:
            raise ValueError("profile_id must be provided when explicit policy profile path is not set.")
        path = _default_policy_profile_path(profile_id)

    if not path.exists():
        raise FileNotFoundError(f"Policy profile not found: {path}")

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load policy profiles.") from exc

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if payload is None:
        payload = {}
    if not isinstance(payload, dict):
        raise ValueError(f"Policy profile must be a mapping: {path}")

    unknown_keys = sorted(set(payload.keys()).difference(_POLICY_PROFILE_ALLOWED_KEYS))
    if unknown_keys:
        raise ValueError(f"Unknown keys in policy profile {path}: {', '.join(unknown_keys)}")

    raw_version = payload.get("profile_version", POLICY_PROFILE_VERSION)
    try:
        profile_version = int(raw_version)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"profile_version must be an integer in {path}") from exc
    if profile_version != POLICY_PROFILE_VERSION:
        raise ValueError(
            f"Unsupported profile_version {profile_version} in {path}. Expected {POLICY_PROFILE_VERSION}."
        )

    defaults = payload.get("defaults", {})
    if not isinstance(defaults, dict):
        raise ValueError(f"defaults must be a mapping in {path}")

    manifest = PolicyProfileManifest(
        profile_id=str(payload.get("profile_id", path.stem)).strip() or path.stem,
        description=str(payload.get("description", "")).strip(),
        defaults=dict(defaults),
        profile_version=profile_version,
    )
    return manifest, path


def apply_policy_profile_defaults(args, defaults: dict[str, Any], parser_defaults: dict[str, Any]) -> list[str]:
    applied: list[str] = []
    for key, value in defaults.items():
        if key not in parser_defaults:
            continue
        current = getattr(args, key, None)
        if current == parser_defaults[key]:
            setattr(args, key, value)
            applied.append(key)
    return applied
