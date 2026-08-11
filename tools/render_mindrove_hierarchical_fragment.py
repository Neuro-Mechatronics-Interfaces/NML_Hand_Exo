from __future__ import annotations

import json
import sys
from pathlib import Path


def main():
    source = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    template = Path(sys.argv[2]).read_text(encoding="utf-8")
    pinch = next(item for item in source["control_triplets"] if item["name"] == "pinch_open_close")
    payload = {
        "classes": source["classes"],
        "full": source["models"],
        "pinch": {key: pinch[key] for key in (
            "raw", "discrete_roll_experts", "continuous_roll", "continuous_roll_pitch"
        )},
        "pinchClasses": pinch["classes"],
        "projection": source["projection"],
    }
    output = template.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":")))
    Path(sys.argv[3]).write_text(output, encoding="utf-8")


if __name__ == "__main__":
    main()
