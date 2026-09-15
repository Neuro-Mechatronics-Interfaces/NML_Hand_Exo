"""Finite plot coordinates with explicit breaks across missing samples."""
import math


def finite_curve_data(times, values):
    """Return x/y and connection flags without passing NaN/Inf to axis scaling.

    A flag connects this point to the next only when both came from consecutive
    samples. Removing invalid points must not draw a line across a data gap.
    """
    x, y, connect = [], [], []
    previous_index = None
    for index, (timestamp, value) in enumerate(zip(times, values)):
        if not math.isfinite(timestamp) or not math.isfinite(value):
            continue
        if connect:
            connect[-1] = index == previous_index + 1
        x.append(timestamp)
        y.append(value)
        connect.append(False)
        previous_index = index
    return x, y, connect
