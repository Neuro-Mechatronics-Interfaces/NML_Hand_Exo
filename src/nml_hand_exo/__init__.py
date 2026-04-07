"""
nml_hand_exo: Python package for controlling the NML HandExo device.

"""

from pathlib import Path
from pkgutil import extend_path


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def _trusted_namespace_paths(paths):
    current_pkg = Path(__file__).resolve().parent
    repo_root = current_pkg.parents[1]
    trusted_roots = [
        repo_root,
        repo_root / "external" / "NeuroBridge",
        repo_root.parent / "NeuroBridge",
    ]

    trusted = []
    for raw_path in paths:
        candidate = Path(raw_path).resolve()
        if candidate == current_pkg or any(_is_within(candidate, root) for root in trusted_roots):
            trusted.append(str(candidate))
    return trusted


__path__ = _trusted_namespace_paths(extend_path(__path__, __name__))

__version__ = "0.0.6"
__author__ = "Neuromechatronics Lab"
__email__ = "neuromech@andrew.cmu.edu"
__license__ = "MIT"
__url__ = "https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo"
__description__ = "Python package for controlling the NML HandExo device."

# Convenience imports for backwards compatibility
# Users should prefer: from nml_hand_exo.interface import HandExo, SerialComm
from .interface import HandExo, SerialComm, TCPComm, FakeHandExo

submodules = [
    'applications',
    'interface',
    'processing',
    'plotting',
    'control',
    'ml',
]

__all__ = submodules + [
    '__version__',
    'HandExo',
    'SerialComm',
    'TCPComm',
    'FakeHandExo',
]

def __dir__():
    return __all__

