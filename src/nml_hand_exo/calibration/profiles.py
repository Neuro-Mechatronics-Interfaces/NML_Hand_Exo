"""Filesystem-backed calibration profile storage without GUI dependencies."""

from __future__ import annotations

import json
from pathlib import Path

from nml_hand_exo._paths import CALIBRATION_PROFILES_DIR


class CalibrationProfileStore:
    """Read and write named, side-aware calibration profiles."""

    def __init__(self, profiles_dir: str | Path = CALIBRATION_PROFILES_DIR):
        self.profiles_dir = Path(profiles_dir)
        self.config_path = self.profiles_dir / "config.json"

    @staticmethod
    def _validate_name(name: str) -> str:
        normalized = str(name).strip()
        if (
            not normalized
            or normalized in (".", "..")
            or "/" in normalized
            or "\\" in normalized
            or Path(normalized).name != normalized
            or normalized.lower() == "config"
        ):
            raise ValueError("Profile name must be a plain, non-empty filename stem")
        return normalized

    def profile_path(self, name: str) -> Path:
        return self.profiles_dir / f"{self._validate_name(name)}.json"

    def list_profiles(self, side: str | None = None) -> list[str]:
        """Return sorted profile names, optionally filtered by hand side."""
        if side not in (None, "left", "right"):
            raise ValueError("side must be 'left', 'right', or None")
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        names = []
        for path in self.profiles_dir.glob("*.json"):
            if path.name == self.config_path.name:
                continue
            if side is not None:
                try:
                    with path.open("r", encoding="utf-8") as profile_file:
                        profile = json.load(profile_file)
                except (OSError, ValueError, json.JSONDecodeError):
                    continue
                if profile.get("side", "right") != side:
                    continue
            names.append(path.stem)
        return sorted(names)

    def load_profile(
        self, name: str | None, side: str = "right"
    ) -> dict | None:
        """Load a profile by name, or the side default when name is None."""
        if name is None:
            name = self.get_default_profile_name(side)
        if name is None:
            return None
        path = self.profile_path(name)
        if not path.exists():
            return None
        with path.open("r", encoding="utf-8") as profile_file:
            return json.load(profile_file)

    def get_default_profile_name(self, side: str = "right") -> str | None:
        """Return the configured default, preserving legacy right-hand fallback."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        if not self.config_path.exists():
            return None
        with self.config_path.open("r", encoding="utf-8") as config_file:
            config = json.load(config_file)
        selected = config.get(f"default_{side}")
        if selected:
            return str(selected)
        if side == "right" and config.get("default"):
            return str(config["default"])
        return None

    def save_profile(self, name: str, data: dict, side: str = "right") -> Path:
        """Save a named profile and ensure its side metadata is authoritative."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        path = self.profile_path(name)
        profile = {**data, "side": side}
        with path.open("w", encoding="utf-8") as profile_file:
            json.dump(profile, profile_file, indent=2)
        return path

    def set_default_profile(self, name: str, side: str = "right") -> None:
        """Set a side-specific default and maintain the legacy right key."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        validated_name = self._validate_name(name)
        config = {}
        if self.config_path.exists():
            with self.config_path.open("r", encoding="utf-8") as config_file:
                config = json.load(config_file)
        config[f"default_{side}"] = validated_name
        if side == "right":
            config["default"] = validated_name
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        with self.config_path.open("w", encoding="utf-8") as config_file:
            json.dump(config, config_file, indent=2)


_DEFAULT_STORE = CalibrationProfileStore()


def profile_path(name: str) -> Path:
    return _DEFAULT_STORE.profile_path(name)


def list_profiles(side: str | None = None) -> list[str]:
    return _DEFAULT_STORE.list_profiles(side)


def load_profile(name: str | None, side: str = "right") -> dict | None:
    return _DEFAULT_STORE.load_profile(name, side)


def get_default_profile_name(side: str = "right") -> str | None:
    return _DEFAULT_STORE.get_default_profile_name(side)


def save_profile(name: str, data: dict, side: str = "right") -> Path:
    return _DEFAULT_STORE.save_profile(name, data, side)


def set_default_profile(name: str, side: str = "right") -> None:
    _DEFAULT_STORE.set_default_profile(name, side)
