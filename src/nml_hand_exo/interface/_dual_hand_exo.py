"""
DualHandExo — manages left and right HandExo instances simultaneously.

Design
------
Each physical exo is its own OpenRB-150 board on its own serial port.
DualHandExo holds up to two HandExo references and presents a unified
API that the GUI (and user scripts) can consume without knowing how many
exos are connected.

Motor naming
------------
Within each HandExo and its calibration profiles, motor names are bare:
``wrist``, ``index``, etc.  DualHandExo prefixes them:

  Left exo  → ``L:wrist``, ``L:index``, …
  Right exo → ``R:wrist``, ``R:index``, …

This makes every aggregated motor name globally unique even when both
exos report the same joint names.

Command routing
---------------
  * ``motor_id = 'all'``       → broadcast to both exos.
  * ``motor_id = 'L:wrist'``   → left exo, bare name ``wrist``.
  * ``motor_id = 'R:index'``   → right exo, bare name ``index``.
  * Bare name (no prefix)       → broadcast to both (best-effort).

Index ordering in aggregated results
-------------------------------------
Left motors occupy indices 0 … N_L-1; right motors N_L … N_L+N_R-1.
This ordering matches the ``info()`` motor dict and the motor widget list
built by the GUI.
"""
from __future__ import annotations

from ._hand_exo import HandExo


class DualHandExo:
    """
    Unified controller for a left and/or right HandExo.

    Either ``left`` or ``right`` may be ``None`` if that side is not
    connected — the class will skip that side gracefully.

    Parameters
    ----------
    left : HandExo or None
        Left-hand exoskeleton instance (already connected).
    right : HandExo or None
        Right-hand exoskeleton instance (already connected).
    """

    _L_PREFIX = "L:"
    _R_PREFIX = "R:"

    def __init__(self, left: HandExo | None, right: HandExo | None):
        self.left  = left
        self.right = right
        # Cached motor counts per side; populated by info().
        # Used to keep global indices consistent when one side fails a poll.
        self._n_motors: dict[str, int] = {}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _active(self) -> list[tuple[str, HandExo]]:
        """Return (side, exo) pairs for every connected side."""
        out = []
        if self.left  is not None: out.append(("left",  self.left))
        if self.right is not None: out.append(("right", self.right))
        return out

    @staticmethod
    def _prefix(side: str) -> str:
        return "L:" if side == "left" else "R:"

    @staticmethod
    def _parse_prefix(name: str) -> tuple[str | None, str]:
        """Split ``'L:wrist'`` → ``('left', 'wrist')``.  No prefix → ``(None, name)``."""
        if name.startswith("L:"):
            return "left", name[2:]
        if name.startswith("R:"):
            return "right", name[2:]
        return None, name

    def _exo(self, side: str) -> HandExo | None:
        return self.left if side == "left" else self.right

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        """Close both serial connections."""
        for _, exo in self._active():
            try:
                exo.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Device info  (mirrors HandExo.info() return shape)
    # ------------------------------------------------------------------

    def info(self) -> dict:
        """
        Return merged device info with side-qualified motor names.

        Left motors occupy indices 0 … N_L-1; right motors follow.
        Each motor dict has an extra ``'side'`` key (``'left'`` or ``'right'``).
        The ``'name'`` key contains the qualified name (e.g. ``'L:wrist'``).
        """
        result: dict = {"motors": {}, "n_motors": 0, "name": "DualHandExo"}
        idx = 0

        for side, exo in self._active():
            prefix = self._prefix(side)
            try:
                side_info = exo.info()
            except Exception:
                continue

            side_motors = side_info.get("motors", {})
            n_side = 0
            for local_key in sorted(side_motors.keys()):
                m = dict(side_motors[local_key])   # shallow copy
                bare_name = m.get("name", f"motor_{idx}")
                m["side"]           = side
                m["bare_name"]      = bare_name
                m["name"]           = prefix + bare_name
                m["qualified_name"] = prefix + bare_name
                result["motors"][idx] = m
                idx += 1
                n_side += 1

            self._n_motors[side] = n_side

        result["n_motors"] = idx
        return result

    # ------------------------------------------------------------------
    # Command routing
    # ------------------------------------------------------------------

    def send_command(self, cmd: str):
        """Broadcast a raw serial command to all connected exos."""
        for _, exo in self._active():
            try:
                exo.send_command(cmd)
            except Exception:
                pass

    def _route(self, method: str, motor_id, *args, **kwargs):
        """
        Route a method call to the correct exo.

        ``motor_id='all'``      → call on every connected exo.
        ``motor_id='L:name'``   → left exo with bare name.
        ``motor_id='R:name'``   → right exo with bare name.
        Bare or int motor_id    → broadcast to both.
        """
        if motor_id == "all":
            for _, exo in self._active():
                try:
                    getattr(exo, method)("all", *args, **kwargs)
                except Exception:
                    pass
            return

        if isinstance(motor_id, str) and (":" in motor_id):
            side, bare = self._parse_prefix(motor_id)
            if side is not None:
                exo = self._exo(side)
                if exo is not None:
                    try:
                        getattr(exo, method)(bare, *args, **kwargs)
                    except Exception:
                        pass
                return

        # Bare name or integer: broadcast to both.
        for _, exo in self._active():
            try:
                getattr(exo, method)(motor_id, *args, **kwargs)
            except Exception:
                pass

    def _collect(self, method: str, *args, **kwargs) -> dict:
        """
        Call a method on all exos and merge results into a unified index dict.

        Assumes the method returns ``{local_int_key: value}`` keyed by the
        motor's local index within that exo.  Remaps to the global index space
        (left first, then right) so the returned dict matches the ``info()``
        ordering.
        """
        result: dict = {}
        global_offset = 0

        for side, exo in self._active():
            n = self._n_motors.get(side, 0)
            try:
                sub = getattr(exo, method)(*args, **kwargs)
                if isinstance(sub, dict):
                    for local_key in sorted(sub.keys()):
                        result[global_offset + int(local_key)] = sub[local_key]
            except Exception:
                pass
            global_offset += n

        return result

    # ------------------------------------------------------------------
    # Motor angle / position
    # ------------------------------------------------------------------

    def get_motor_angle(self, motor_id="all"):
        if motor_id == "all":
            return self._collect("get_motor_angle", "all")
        side, bare = self._parse_prefix(str(motor_id))
        exo = self._exo(side or "right")
        return exo.get_motor_angle(bare) if exo else None

    def set_motor_angle(self, motor_id, angle: float):
        self._route("set_motor_angle", motor_id, angle)

    def get_absolute_motor_angle(self, motor_id="all"):
        if motor_id == "all":
            return self._collect("get_absolute_motor_angle", "all")
        side, bare = self._parse_prefix(str(motor_id))
        exo = self._exo(side or "right")
        return exo.get_absolute_motor_angle(bare) if exo else None

    def set_absolute_motor_angle(self, motor_id, angle: float):
        self._route("set_absolute_motor_angle", motor_id, angle)

    # ------------------------------------------------------------------
    # Torque / current telemetry
    # ------------------------------------------------------------------

    def get_motor_torque(self, motor_id="all"):
        if motor_id == "all":
            return self._collect("get_motor_torque", "all")
        side, bare = self._parse_prefix(str(motor_id))
        exo = self._exo(side or "right")
        return exo.get_motor_torque(bare) if exo else None

    def get_motor_current(self, motor_id="all"):
        if motor_id == "all":
            return self._collect("get_motor_current", "all")
        side, bare = self._parse_prefix(str(motor_id))
        exo = self._exo(side or "right")
        return exo.get_motor_current(bare) if exo else None

    # ------------------------------------------------------------------
    # Enable / disable / home
    # ------------------------------------------------------------------

    def enable_motor(self, motor_id="all"):
        self._route("enable_motor", motor_id)

    def disable_motor(self, motor_id="all"):
        self._route("disable_motor", motor_id)

    def home(self, motor_id="all"):
        self._route("home", motor_id)

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def apply_calibration(
        self,
        profile_or_path: str | None = None,
        profiles_dir: str | None = None,
        side: str | None = None,
    ):
        """
        Apply a calibration profile to one or both exos.

        Parameters
        ----------
        profile_or_path : str or None
            Profile name (e.g. ``"zach left new"``) or full file path.
            ``None`` → load the side-specific default from ``config.json``.
        profiles_dir : str or None
            Directory containing profile JSON files.  Defaults to the
            repo's ``examples/calibration/profiles/`` directory.
        side : 'left', 'right', or None
            Which exo to apply the profile to.  When ``None``, the method
            reads the profile's ``side`` field; if absent, defaults to
            ``'right'``.

        Raises
        ------
        RuntimeError
            If the target exo is not connected.
        """
        import json, os

        if profiles_dir is None:
            repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.abspath(__file__)))))
            profiles_dir = os.path.join(
                repo_root, "examples", "calibration", "profiles"
            )

        # Resolve filepath
        if profile_or_path is None:
            config_path = os.path.join(profiles_dir, "config.json")
            with open(config_path) as f:
                cfg = json.load(f)
            # Side-specific default, then legacy "default"
            key = f"default_{side}" if side else None
            profile_name = (cfg.get(key) if key else None) or cfg.get("default")
            if not profile_name:
                raise ValueError(
                    f"No default profile found for side='{side}'. "
                    "Set one with calibrate_exo.py --set-default."
                )
            filepath = os.path.join(profiles_dir, f"{profile_name}.json")
        elif os.path.isfile(profile_or_path):
            filepath = profile_or_path
        else:
            filepath = os.path.join(profiles_dir, f"{profile_or_path}.json")

        with open(filepath) as f:
            cal = json.load(f)

        # Determine target side
        target_side = side or cal.get("side", "right")
        exo = self._exo(target_side)
        if exo is None:
            raise RuntimeError(
                f"No '{target_side}' exo is connected. "
                "Check the port configuration in dual mode."
            )

        exo.apply_calibration(filepath, profiles_dir)

    # ------------------------------------------------------------------
    # Low-level calibration setters (pass-through routing)
    # ------------------------------------------------------------------

    def set_zero_offset(self, motor_id, offset: float):
        self._route("set_zero_offset", motor_id, offset)

    def set_flip(self, motor_id, flip: bool):
        self._route("set_flip", motor_id, flip)

    def set_motor_limits(self, motor_id, lo: float, hi: float):
        self._route("set_motor_limits", motor_id, lo, hi)

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def set_debug(self, enable: bool):
        for _, exo in self._active():
            try:
                exo.set_debug(enable)
            except Exception:
                pass

    def set_exo_mode(self, mode: str):
        for _, exo in self._active():
            try:
                exo.set_exo_mode(mode)
            except Exception:
                pass
