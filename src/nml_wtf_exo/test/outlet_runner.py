# nml/test/outlet_runner.py
from __future__ import annotations
import argparse
import json
import os
import sys
import pylsl

from nml_wtf_exo.utils.PhraseHandler import PhraseHandler

StreamInfo   = pylsl.StreamInfo
StreamOutlet = pylsl.StreamOutlet
cf_string = getattr(pylsl, "cf_string", getattr(pylsl, "CF_STRING", None))
if cf_string is None:
    raise ImportError("Could not find pylsl.cf_string / CF_STRING. Upgrade pylsl.")
IRREGULAR_RATE = getattr(pylsl, "IRREGULAR_RATE", 0.0)

def read_phrases(path: str):
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Phrases file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return [line.rstrip("\n") for line in f if line.strip()]

def main():
    ap = argparse.ArgumentParser(description="LSL Typing Outlet (JSON key events)")
    ap.add_argument("--file", default="resources/phrases.txt")
    ap.add_argument("--name", default="KeyboardEvents")
    ap.add_argument("--type", default="Markers")
    ap.add_argument("--wpm", type=float, default=60.0)
    ap.add_argument("--cv", type=float, default=0.30)
    ap.add_argument("--word-pause", type=float, default=0.0)
    ap.add_argument("--error-rate", type=float, default=0.02,
                    help="Probability an intended char starts a typo episode")
    ap.add_argument("--error-burst-p", type=float, default=0.70,
                    help="Geometric stop probability for typo length (E[K]=1/p)")
    args = ap.parse_args()

    phrases = read_phrases(args.file)

    info = StreamInfo(args.name, args.type, 1, IRREGULAR_RATE, cf_string, "json-key-events")
    info.desc().append_child_value("producer", "nml-typist")
    outlet = StreamOutlet(info)

    def emit(ev: dict):
        outlet.push_sample([json.dumps(ev, separators=(",", ":"))])

    handler = PhraseHandler(
        phrases=phrases,
        emit=emit,
        wpm=args.wpm,
        cv=args.cv,
        word_pause_mean=args.word_pause,
        error_rate=args.error_rate,
        error_burst_p=args.error_burst_p,
    )

    print(f"Streaming JSON key events on '{args.name}' (type={args.type}).")
    print(f"WPM={args.wpm:g}, CV={args.cv:g}, word_pause={args.word_pause:g}, "
          f"error_rate={args.error_rate:g}, error_burst_p={args.error_burst_p:g}")
    handler.type_all(newline_between=True)
    print(f"Done. total_error_keys={handler.total_error_keys}")

if __name__ == "__main__":
    sys.exit(main())
