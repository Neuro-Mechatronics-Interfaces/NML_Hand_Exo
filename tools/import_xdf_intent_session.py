"""Build a reloadable EMG intent session from event-marked MindRove XDF files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from nml_hand_exo.decoding import import_xdf_session


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path", type=Path, help="One XDF file or a directory containing XDF files")
    parser.add_argument("output", type=Path)
    parser.add_argument("--exclude", action="append", default=[], help="XDF basename to leave out; repeat as needed")
    parser.add_argument("--participant", default="jonathan")
    args = parser.parse_args()

    excluded = {Path(value).name.lower() for value in args.exclude}
    candidates = [args.input_path] if args.input_path.is_file() else sorted(args.input_path.glob("*.xdf"))
    files = [path for path in candidates if path.suffix.lower() == ".xdf" and path.name.lower() not in excluded]
    session, summary = import_xdf_session(
        files,
        participant_id=args.participant,
        progress=lambda index, total, path: print(f"[{index}/{total}] {path.name}"),
    )
    session.save(args.output)
    print(json.dumps({"output": str(args.output), "excluded": sorted(excluded), **summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
