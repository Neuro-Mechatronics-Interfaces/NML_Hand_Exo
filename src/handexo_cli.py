"""Backward-compatible module wrapper for the installed ``handexo`` command."""

from __future__ import annotations

from nml_hand_exo.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
