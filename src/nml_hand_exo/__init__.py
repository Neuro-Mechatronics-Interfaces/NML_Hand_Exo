"""
nml_hand_exo: Python package for controlling the NML HandExo device.

"""

__version__ = "0.2.17"
__author__ = "Neuromechatronics Lab"
__email__ = "neuromech@andrew.cmu.edu"
__license__ = "MIT"
__url__ = "https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo"
__description__ = "Python package for controlling the NML HandExo device."

# Convenience imports for backwards compatibility
# Users should prefer: from nml_hand_exo.interface import HandExo, SerialComm
from .interface import HandExo, SerialComm, DualSerialComm, TCPComm, FakeHandExo

__all__ = [
    '__version__',
    'HandExo',
    'SerialComm',
    'DualSerialComm',
    'TCPComm',
    'FakeHandExo',
]

def __dir__():
    return sorted(__all__)

