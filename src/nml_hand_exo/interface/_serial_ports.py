"""Helpers for presenting pyserial port metadata."""

from __future__ import annotations

import re
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


def usb_interface_index(port: Any) -> int | None:
    """Best-effort USB interface index (``MI_xx``) for a composite CDC port.

    A dual-CDC device exposes two COM ports that differ only by interface index
    (e.g. ``MI_00`` = command, ``MI_02`` = telemetry).  Returns the integer index
    or ``None`` when it cannot be determined.
    """
    for attr in ("hwid", "location", "device"):
        val = getattr(port, attr, "") or ""
        m = re.search(r"MI_(\d+)", val, re.IGNORECASE)
        if m:
            return int(m.group(1))
    # Some platforms expose the interface as a trailing ``.N`` in ``location``.
    loc = getattr(port, "location", "") or ""
    m = re.search(r"[.:](\d+)$", loc)
    if m:
        return int(m.group(1))
    return None


def find_cdc_sibling(device: str, ports: list | None = None) -> tuple[str, str] | None:
    """Pair the two USB-CDC interfaces of one physical device.

    Given one COM port ``device`` string, finds the other CDC interface of the
    SAME physical device (matching ``serial_number`` + VID/PID, different
    interface) and returns ``(cmd_device, telem_device)`` ordered so the lower
    USB interface index is the command port.  The direction is only a hint —
    :class:`~nml_hand_exo.interface.DualSerialComm` probes and corrects it at
    connect time.  Returns ``None`` if no unambiguous sibling is found.
    """
    from serial.tools import list_ports

    ports = list(ports if ports is not None else list_ports.comports())
    selected = next((p for p in ports if p.device == device), None)
    if selected is None:
        return None

    serial_number = getattr(selected, "serial_number", None)
    vid = getattr(selected, "vid", None)
    pid = getattr(selected, "pid", None)
    if not serial_number or vid is None or pid is None:
        return None

    group = [
        p for p in ports
        if getattr(p, "serial_number", None) == serial_number
        and getattr(p, "vid", None) == vid
        and getattr(p, "pid", None) == pid
    ]
    if len(group) < 2:
        return None

    # Order by USB interface index when available, else by COM device name.
    def sort_key(p):
        idx = usb_interface_index(p)
        return (0, idx) if idx is not None else (1, p.device)

    group.sort(key=sort_key)
    cmd_device = group[0].device
    telem_device = next((p.device for p in group if p.device != cmd_device), None)
    if telem_device is None:
        return None
    return (cmd_device, telem_device)
