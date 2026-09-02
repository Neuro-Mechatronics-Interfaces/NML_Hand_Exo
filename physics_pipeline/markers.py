"""Backward-compatible parsing and formatting for NML task markers."""

from __future__ import annotations

from collections.abc import Mapping


RESERVED = ("|", "\n", "\r")


def parse_marker(value: str) -> dict[str, str]:
    parts = str(value).split("|")
    fields = {"event": parts[0].strip()}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, item = part.split("=", 1)
        fields[key.strip()] = item.strip()
    return fields


def format_marker(event: str, fields: Mapping[str, object] | None = None) -> str:
    event = _safe_token("event", event)
    parts = [event]
    for key, value in (fields or {}).items():
        clean_key = _safe_token("marker key", key)
        clean_value = _safe_token(f"marker value for {clean_key}", value)
        parts.append(f"{clean_key}={clean_value}")
    return "|".join(parts)


def _safe_token(label: str, value: object) -> str:
    text = str(value).strip()
    if not text:
        raise ValueError(f"{label} cannot be empty")
    if any(token in text for token in RESERVED):
        raise ValueError(f"{label} contains a reserved marker character")
    return text
