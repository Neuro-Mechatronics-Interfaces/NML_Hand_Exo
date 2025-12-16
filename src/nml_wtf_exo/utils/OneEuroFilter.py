import math

class OneEuro:
    """
    NaN-safe One Euro filter.

    Args:
        freq (float): expected sample rate (Hz)
        min_cutoff (float): base cutoff (Hz)
        beta (float): cutoff slope vs. |dx|
        d_cutoff (float): derivative cutoff (Hz)
        nan_policy (str): 'gap' -> return NaN on invalid input (no state update)
                          'hold' -> return last estimate on invalid input (no state update)
        min_cutoff_floor (float): lower bound to keep alphas stable
    """
    def __init__(self, freq, min_cutoff=1.0, beta=0.05, d_cutoff=6.0,
                 nan_policy="gap", min_cutoff_floor=1e-6):
        self.freq = max(1e-9, float(freq))
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.nan_policy = nan_policy  # 'gap' or 'hold'
        self.min_cutoff_floor = float(min_cutoff_floor)
        self._x = None   # last filtered value
        self._dx = None  # last filtered derivative

    def reset(self, x=None):
        """Reset internal state; optionally seed with initial value."""
        self._x = float(x) if (x is not None and math.isfinite(float(x))) else None
        self._dx = None

    def set_freq(self, freq):
        """Update assumed sample rate (Hz)."""
        self.freq = max(1e-9, float(freq))

    def _alpha(self, cutoff):
        cutoff = max(self.min_cutoff_floor, float(cutoff))
        te = 1.0 / self.freq
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / te)

    def filter(self, x):
        # Parse & check finiteness without throwing
        try:
            xf = float(x)
        except Exception:
            xf = float('nan')

        if not math.isfinite(xf):
            # Do NOT update state on invalid input
            if self.nan_policy == "hold" and self._x is not None:
                return self._x
            return float('nan')

        # First valid sample: seed state and return it directly
        if self._x is None:
            self._x = xf
            self._dx = 0.0
            return self._x

        # Derivative (raw)
        dx = (xf - self._x) * self.freq
        # Low-pass derivative
        a_d = self._alpha(self.d_cutoff)
        self._dx = a_d * dx + (1.0 - a_d) * (self._dx if self._dx is not None else dx)

        # Adaptive cutoff
        cutoff = max(self.min_cutoff_floor, self.min_cutoff + self.beta * abs(self._dx))
        a = self._alpha(cutoff)

        # Low-pass signal
        self._x = a * xf + (1.0 - a) * self._x
        return self._x
