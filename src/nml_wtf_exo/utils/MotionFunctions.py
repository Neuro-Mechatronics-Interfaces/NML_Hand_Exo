import math
import random
import json

# ---------------------------------------------------------------------
# Motion Function Base Class
# ---------------------------------------------------------------------
class MotionFunctionType:
    """
    Base class for joint motion generators.
    frequency is expressed in RPM (revolutions per minute); internally we use Hz.
    """
    type_name = "base"

    def __init__(self, amplitude=10.0, frequency=20.0):
        self.amplitude = float(amplitude)
        self.frequency = float(frequency)     # RPM
        self._frequency = self.frequency / 60.0  # Hz
        self.t = 0.0

    def step(self, dt: float) -> float:
        """Override in subclasses. Return offset (deg) from home."""
        return 0.0

    def set_params(self, amplitude=None, frequency=None):
        if amplitude is not None:
            self.amplitude = float(amplitude)
        if frequency is not None:
            self.frequency = float(frequency)
            self._frequency = self.frequency / 60.0

    # ---- config helpers ----
    def to_config(self) -> dict:
        """Return a dict describing this motion (JSON-serializable)."""
        return {
            "type": self.type_name,
            "amplitude": self.amplitude,
            "frequency": self.frequency,
        }

    @classmethod
    def from_config(cls, cfg: dict) -> "MotionFunctionType":
        """
        Construct a motion object from a config dict.
        Dispatches to the proper subclass based on cfg['type'].
        """
        mtype = cfg.get("type", "sine")
        MotionCls = MOTION_CLASS_REGISTRY.get(mtype, MotionSine)
        amp = cfg.get("amplitude", 10.0)
        freq = cfg.get("frequency", 20.0)
        obj = MotionCls(amplitude=amp, frequency=freq)

        # Subclasses may have extra fields; let them post-process if needed.
        extra = {k: v for k, v in cfg.items() if k not in ("type", "amplitude", "frequency")}
        if hasattr(obj, "load_extra_config"):
            obj.load_extra_config(extra)

        return obj

    def to_json(self) -> str:
        """Return JSON string representation of this motion."""
        return json.dumps(self.to_config(), indent=2)

    @staticmethod
    def from_json(js: str) -> "MotionFunctionType":
        cfg = json.loads(js)
        return MotionFunctionType.from_config(cfg)


# ---------------------------------------------------------------------
# Sine Wave Motion
# ---------------------------------------------------------------------
class MotionSine(MotionFunctionType):
    type_name = "sine"

    def step(self, dt: float) -> float:
        self.t += dt
        phase = 2 * math.pi * self._frequency * self.t
        return self.amplitude * math.sin(phase)


# ---------------------------------------------------------------------
# Triangle Wave Motion
# ---------------------------------------------------------------------
class MotionTriangle(MotionFunctionType):
    type_name = "triangle"

    def step(self, dt: float) -> float:
        self.t += dt
        # phase in [0,1)
        phase = (self._frequency * self.t) % 1.0
        # triangle from -1 to 1
        tri = 4.0 * phase
        if tri > 2.0:
            tri = 4.0 - tri
        tri -= 1.0  # shift to [-1, 1]
        return self.amplitude * tri


# ---------------------------------------------------------------------
# Alternating Step (+A / -A)
# ---------------------------------------------------------------------
class MotionAlternatingStep(MotionFunctionType):
    type_name = "alt_step"

    def __init__(self, amplitude=10.0, frequency=20.0):
        super().__init__(amplitude=amplitude, frequency=frequency)
        self._state = 1.0  # start at +A

    def step(self, dt: float) -> float:
        self.t += dt
        # flip sign every half-period
        half_period = 0.5 / max(self._frequency, 1e-6)
        while self.t >= half_period:
            self.t -= half_period
            self._state *= -1.0
        return self.amplitude * self._state


# ---------------------------------------------------------------------
# White Noise Motion
# ---------------------------------------------------------------------
class MotionWhiteNoise(MotionFunctionType):
    type_name = "white_noise"

    def step(self, dt: float) -> float:
        # frequency can be thought of as how "fast" we allow changes,
        # but for now we just ignore it and generate iid samples
        return self.amplitude * (2.0 * random.random() - 1.0)


# ---------------------------------------------------------------------
# Pink Noise (approximate 1/f via 1-pole filter)
# ---------------------------------------------------------------------
class MotionPinkNoise(MotionFunctionType):
    type_name = "pink_noise"

    def __init__(self, amplitude=10.0, frequency=20.0):
        super().__init__(amplitude=amplitude, frequency=frequency)
        self.y = 0.0
        self.alpha = 0.9  # smoothing factor; closer to 1 = slower

    def load_extra_config(self, extra: dict):
        a = extra.get("alpha", None)
        if a is not None:
            self.alpha = float(a)

    def to_config(self) -> dict:
        base = super().to_config()
        base["alpha"] = self.alpha
        return base

    def step(self, dt: float) -> float:
        # approximate pink noise: filtered white noise
        w = 2.0 * random.random() - 1.0
        self.y = self.alpha * self.y + (1.0 - self.alpha) * w
        return self.amplitude * self.y


# Global registry for dispatch
MOTION_CLASS_REGISTRY = {
    "sine": MotionSine,
    "triangle": MotionTriangle,
    "alt_step": MotionAlternatingStep,
    "white_noise": MotionWhiteNoise,
    "pink_noise": MotionPinkNoise,
}
