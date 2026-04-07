from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEPLOYMENT_BUNDLE_VERSION = 1
_BUNDLE_ALLOWED_KEYS = {
    "bundle_version",
    "bundle_id",
    "description",
    "adapter",
    "signature",
    "policy_profile",
    "skill_packs",
    "telemetry",
    "runtime_defaults",
    "tags",
}
_ADAPTER_ALLOWED_KEYS = {"id", "config"}
_SIGNATURE_ALLOWED_KEYS = {"required", "sha256", "file"}
_TELEMETRY_ALLOWED_KEYS = {"schema_version", "strict"}


@dataclass
class DeploymentBundleManifest:
    bundle_id: str
    description: str = ""
    adapter_id: str = ""
    adapter_config_path: str = ""
    signature_required: bool = False
    signature_verified: bool = False
    policy_profile_path: str = ""
    skill_pack_paths: list[str] = field(default_factory=list)
    telemetry_schema_version: str = ""
    telemetry_schema_strict: bool = False
    runtime_defaults: dict[str, Any] = field(default_factory=dict)
    tags: list[str] = field(default_factory=list)
    bundle_version: int = DEPLOYMENT_BUNDLE_VERSION


def _default_bundle_path(bundle_id: str) -> Path:
    return Path(__file__).resolve().parents[3] / "config" / "bundles" / f"{bundle_id}.yaml"


def list_available_bundles() -> list[str]:
    root = Path(__file__).resolve().parents[3] / "config" / "bundles"
    if not root.exists():
        return []
    return sorted(path.stem for path in root.glob("*.yaml"))


def _sha256_hex(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().lower()


def _resolve_relative_path(value: str, base_dir: Path) -> Path:
    candidate = Path(value)
    if candidate.is_absolute():
        return candidate.resolve()
    return (base_dir / candidate).resolve()


def _load_expected_signature(signature_payload: dict[str, Any], bundle_path: Path) -> str:
    signature_value = str(signature_payload.get("sha256", "")).strip().lower()
    if signature_value:
        return signature_value

    signature_file = str(signature_payload.get("file", "")).strip()
    if not signature_file:
        return ""
    resolved_signature_file = _resolve_relative_path(signature_file, bundle_path.parent)
    if not resolved_signature_file.exists():
        raise FileNotFoundError(
            f"Bundle signature file not found for {bundle_path}: {resolved_signature_file}"
        )
    first_token = resolved_signature_file.read_text(encoding="utf-8").strip().split()
    return first_token[0].lower() if first_token else ""


def load_bundle_manifest(bundle_id: str = "", explicit_path: str | None = None) -> tuple[DeploymentBundleManifest, Path]:
    if explicit_path:
        path = Path(explicit_path).expanduser().resolve()
    else:
        if not bundle_id:
            raise ValueError("bundle_id must be provided when explicit bundle path is not set.")
        path = _default_bundle_path(bundle_id)

    if not path.exists():
        raise FileNotFoundError(f"Deployment bundle not found: {path}")

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load deployment bundles.") from exc

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if payload is None:
        payload = {}
    if not isinstance(payload, dict):
        raise ValueError(f"Deployment bundle must be a mapping: {path}")

    unknown_keys = sorted(set(payload.keys()).difference(_BUNDLE_ALLOWED_KEYS))
    if unknown_keys:
        raise ValueError(f"Unknown keys in deployment bundle {path}: {', '.join(unknown_keys)}")

    raw_version = payload.get("bundle_version", DEPLOYMENT_BUNDLE_VERSION)
    try:
        bundle_version = int(raw_version)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"bundle_version must be an integer in {path}") from exc
    if bundle_version != DEPLOYMENT_BUNDLE_VERSION:
        raise ValueError(
            f"Unsupported bundle_version {bundle_version} in {path}. Expected {DEPLOYMENT_BUNDLE_VERSION}."
        )

    adapter_payload = payload.get("adapter", {})
    if not isinstance(adapter_payload, dict):
        raise ValueError(f"adapter must be a mapping in {path}")
    unknown_adapter_keys = sorted(set(adapter_payload.keys()).difference(_ADAPTER_ALLOWED_KEYS))
    if unknown_adapter_keys:
        raise ValueError(
            f"Unknown adapter keys in deployment bundle {path}: {', '.join(unknown_adapter_keys)}"
        )

    adapter_id = str(adapter_payload.get("id", "")).strip().lower()
    if not adapter_id:
        raise ValueError(f"adapter.id is required in deployment bundle {path}")

    adapter_config = str(adapter_payload.get("config", "")).strip()
    if adapter_config and not Path(adapter_config).is_absolute():
        adapter_config = str((path.parent / adapter_config).resolve())

    signature_payload = payload.get("signature", {})
    if signature_payload is None:
        signature_payload = {}
    if not isinstance(signature_payload, dict):
        raise ValueError(f"signature must be a mapping in {path}")
    unknown_signature_keys = sorted(set(signature_payload.keys()).difference(_SIGNATURE_ALLOWED_KEYS))
    if unknown_signature_keys:
        raise ValueError(
            f"Unknown signature keys in deployment bundle {path}: {', '.join(unknown_signature_keys)}"
        )

    signature_required = bool(signature_payload.get("required", False))
    expected_signature = _load_expected_signature(signature_payload, path)
    signature_verified = False
    if expected_signature:
        observed_signature = _sha256_hex(path)
        signature_verified = observed_signature == expected_signature
        if signature_required and not signature_verified:
            raise ValueError(
                f"Bundle signature mismatch for {path}. expected={expected_signature} observed={observed_signature}"
            )
    elif signature_required:
        raise ValueError(
            f"signature.required is true but no signature value was provided in {path}"
        )

    policy_profile = str(payload.get("policy_profile", "")).strip()
    policy_profile_path = ""
    if policy_profile:
        resolved_policy_path = _resolve_relative_path(policy_profile, path.parent)
        if not resolved_policy_path.exists():
            raise FileNotFoundError(
                f"Policy profile referenced by deployment bundle not found: {resolved_policy_path}"
            )
        policy_profile_path = str(resolved_policy_path)

    skill_pack_payload = payload.get("skill_packs", [])
    if not isinstance(skill_pack_payload, list):
        raise ValueError(f"skill_packs must be a list in {path}")
    skill_pack_paths: list[str] = []
    for raw_skill_pack in skill_pack_payload:
        skill_pack_ref = str(raw_skill_pack).strip()
        if not skill_pack_ref:
            continue
        resolved_skill_pack = _resolve_relative_path(skill_pack_ref, path.parent)
        if not resolved_skill_pack.exists():
            raise FileNotFoundError(
                f"Skill pack referenced by deployment bundle not found: {resolved_skill_pack}"
            )
        skill_pack_paths.append(str(resolved_skill_pack))

    telemetry_payload = payload.get("telemetry", {})
    if telemetry_payload is None:
        telemetry_payload = {}
    if not isinstance(telemetry_payload, dict):
        raise ValueError(f"telemetry must be a mapping in {path}")
    unknown_telemetry_keys = sorted(set(telemetry_payload.keys()).difference(_TELEMETRY_ALLOWED_KEYS))
    if unknown_telemetry_keys:
        raise ValueError(
            f"Unknown telemetry keys in deployment bundle {path}: {', '.join(unknown_telemetry_keys)}"
        )
    telemetry_schema_version = str(telemetry_payload.get("schema_version", "")).strip()
    telemetry_schema_strict = bool(telemetry_payload.get("strict", False))
    if telemetry_schema_strict and not telemetry_schema_version:
        raise ValueError(
            f"telemetry.strict requires telemetry.schema_version in deployment bundle {path}"
        )

    runtime_defaults = payload.get("runtime_defaults", {})
    if not isinstance(runtime_defaults, dict):
        raise ValueError(f"runtime_defaults must be a mapping in {path}")

    tags_payload = payload.get("tags", [])
    if not isinstance(tags_payload, list):
        raise ValueError(f"tags must be a list in {path}")

    manifest = DeploymentBundleManifest(
        bundle_id=str(payload.get("bundle_id", path.stem)).strip() or path.stem,
        description=str(payload.get("description", "")).strip(),
        adapter_id=adapter_id,
        adapter_config_path=adapter_config,
        signature_required=signature_required,
        signature_verified=signature_verified,
        policy_profile_path=policy_profile_path,
        skill_pack_paths=skill_pack_paths,
        telemetry_schema_version=telemetry_schema_version,
        telemetry_schema_strict=telemetry_schema_strict,
        runtime_defaults=dict(runtime_defaults),
        tags=[str(tag).strip() for tag in tags_payload if str(tag).strip()],
        bundle_version=bundle_version,
    )
    return manifest, path


def apply_bundle_defaults(args, runtime_defaults: dict[str, Any], parser_defaults: dict[str, Any]) -> list[str]:
    applied: list[str] = []
    for key, value in runtime_defaults.items():
        if key not in parser_defaults:
            continue
        current = getattr(args, key, None)
        if current == parser_defaults[key]:
            setattr(args, key, value)
            applied.append(key)
    return applied
