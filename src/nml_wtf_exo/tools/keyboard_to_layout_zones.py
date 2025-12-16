#!/usr/bin/env python3
"""
Convert a pixel-based keyboard_layout.json into a unit-scaled "zones" JSON
compatible with the Unity control zone format shown in the prompt.

Usage (from repo root):
  python tools/keyboard_layout_to_zones.py \
      --in resources/keyboard_layout.json \
      --out resources/control_zones.json \
      --imgw 990 --imgh 255 \
      --x-min 0.05 --x-max 0.95 \
      --y-min 0.10 --y-max 0.40 \
      --color "#B0B0B0FF"

Notes:
- IDs are ASCII codes where possible (letters, digits, punctuation, and a few
  special keys via ASCII control codes). Keys with no ASCII mapping are skipped.
- All numeric fields are rounded to 3 decimals.
"""

from __future__ import annotations
import argparse
import json
import os
import sys
from typing import Dict, Tuple, List, Optional

# Map common non-single-char key names to ASCII control chars
ASCII_MAP: Dict[str, str] = {
    "space": " ",
    "enter": "\n", "return": "\n",
    "tab": "\t",
    "backspace": "\b",
    "esc": "\x1b", "escape": "\x1b",
    "del": "\x7f", "delete": "\x7f",
}

def r3(x: float) -> float:
    """Round to 3 decimals (as float)."""
    return float(f"{x:.3f}")

def key_to_ascii_id(key: str) -> Optional[int]:
    """
    Return ASCII integer for a key where possible.
    - Single-character keys: ord(key)
    - Known specials (via ASCII_MAP): ord(mapped_char)
    - Otherwise: None (skip)
    """
    k = key.strip().lower()
    if len(k) == 1:
        return ord(k)
    if k in ASCII_MAP:
        return ord(ASCII_MAP[k])
    return None

def normalize_rect(
    x: float, y: float, w: float, h: float,
    imgw: float, imgh: float,
    x_min: float, x_max: float,
    y_min: float, y_max: float
) -> Tuple[float, float, float, float]:
    """
    Map pixel rect [x,y,w,h] from image size (imgw,imgh) into unit space:
      X' = x_min + (x/imgw) * (x_max - x_min)
      Y' = y_min + (y/imgh) * (y_max - y_min)
      W' = (w/imgw) * (x_max - x_min)
      H' = (h/imgh) * (y_max - y_min)
    """
    xr = x_min + (x / imgw) * (x_max - x_min)
    yr = y_min + (y / imgh) * (y_max - y_min)
    wr = (w / imgw) * (x_max - x_min)
    hr = (h / imgh) * (y_max - y_min)
    return (r3(xr), r3(yr), r3(wr), r3(hr))

def load_layout(path: str) -> Dict[str, List[float]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    keys = data.get("keys") or {}
    # Expect keys: { "a": [x, y, w, h], ... }
    return {str(k).lower(): list(map(float, v)) for k, v in keys.items()}

def convert(
    layout_path: str,
    out_path: str,
    imgw: float,
    imgh: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    color: str,
    name: str = "Layout",
    version: str = "1"
) -> Dict:
    key_rects = load_layout(layout_path)

    zones = []
    skipped: List[str] = []

    # Keep things stable by sorting keys by (y, x) then name
    def sort_key(item):
        k, rect = item
        x, y, w, h = rect
        return (y, x, k)

    for k, rect in sorted(key_rects.items(), key=sort_key):
        if len(rect) != 4:
            skipped.append(k)
            continue

        ascii_id = key_to_ascii_id(k)
        if ascii_id is None:
            skipped.append(k)
            continue

        x, y, w, h = rect
        xn, yn, wn, hn = normalize_rect(
            x, y, w, h, imgw, imgh, x_min, x_max, y_min, y_max
        )

        zones.append({
            "id": int(ascii_id),
            "name": k, 
            "x": xn, "y": yn, "w": wn, "h": hn,
            "color": color
        })

    layout = {"name": name, "version": version, "zones": zones}

    # Write output
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(layout, f, ensure_ascii=False, indent=4)

    # Report skipped keys (non-ASCII)
    if skipped:
        sys.stderr.write(
            f"[INFO] Skipped {len(skipped)} key(s) with no ASCII mapping: "
            + ", ".join(skipped) + "\n"
        )

    return layout

def main():
    ap = argparse.ArgumentParser(description="Convert keyboard_layout.json to unit-scaled control zones.")
    ap.add_argument("--in", dest="inp", default="resources/keyboard_layout.json",
                    help="Path to input keyboard_layout.json")
    ap.add_argument("--out", dest="out", default="resources/control_zones.json",
                    help="Path to output zones JSON")
    ap.add_argument("--imgw", type=float, default=990.0, help="Image width in pixels")
    ap.add_argument("--imgh", type=float, default=255.0, help="Image height in pixels")
    ap.add_argument("--x-min", type=float, default=0.05)
    ap.add_argument("--x-max", type=float, default=0.95)
    ap.add_argument("--y-min", type=float, default=0.50)
    ap.add_argument("--y-max", type=float, default=0.90)
    ap.add_argument("--color", type=str, default="#B0B0B0FF",
                    help="RGBA hex color for zones")
    ap.add_argument("--name", type=str, default="Layout")
    ap.add_argument("--version", type=str, default="1")
    args = ap.parse_args()

    convert(
        layout_path=args.inp,
        out_path=args.out,
        imgw=args.imgw,
        imgh=args.imgh,
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        color=args.color,
        name=args.name,
        version=args.version,
    )

if __name__ == "__main__":
    sys.exit(main())
