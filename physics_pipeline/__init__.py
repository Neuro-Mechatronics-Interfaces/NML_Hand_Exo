"""Physics-informed EMG/exoskeleton recording and modeling tools.

This top-level package is intentionally isolated from the installed
``nml_hand_exo`` control package.  Hardware control remains in the core package;
this package owns recording contracts, XDF tooling, offline synchronization,
and research models.
"""

from .contracts import (
    EXO_STATE_SCHEMA,
    EXO_STATE_STREAM_NAME,
    EXO_STATE_STREAM_TYPE,
    ExoMotorDescriptor,
    NumericChannelSpec,
    build_exo_state_channels,
)
from .manifest import SessionManifest

__all__ = [
    "EXO_STATE_SCHEMA",
    "EXO_STATE_STREAM_NAME",
    "EXO_STATE_STREAM_TYPE",
    "ExoMotorDescriptor",
    "NumericChannelSpec",
    "SessionManifest",
    "build_exo_state_channels",
]
