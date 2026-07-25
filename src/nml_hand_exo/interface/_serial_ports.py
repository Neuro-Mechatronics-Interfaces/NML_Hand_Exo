"""Helpers for presenting pyserial port metadata."""

from __future__ import annotations

from typing import Any


def format_port_label(port: Any) -> str:
    """Build a readable label from a pyserial ``ListPortInfo`` object."""
    description = port.description or ""
    hwid = getattr(port, "hwid", "") or ""
    desc_lower = description.lower()
    hwid_lower = hwid.lower()

    tags = []
    if "bluetooth" in desc_lower or "rfcomm" in hwid_lower or "bthenum" in hwid_lower:
        tags.append("BT")
    if "usb serial device" in desc_lower or "usb" in desc_lower:
        tags.append("USB")
    if "nml_exo" in desc_lower or "nml_exo" in hwid_lower:
        tags.append("NML_EXO")

    parts = [port.device]
    if tags:
        parts.append(f"[{', '.join(tags)}]")
    if description:
        parts.append(description)
    if getattr(port, "manufacturer", None):
        parts.append(port.manufacturer)
    if getattr(port, "serial_number", None):
        parts.append(f"SN:{port.serial_number}")
    if getattr(port, "vid", None) is not None and getattr(port, "pid", None) is not None:
        parts.append(f"VID:{port.vid:04X} PID:{port.pid:04X}")
    if hwid:
        parts.append(hwid)
    return " - ".join(parts)
