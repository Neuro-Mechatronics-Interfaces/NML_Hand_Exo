from ._interfaces import (
    BaseComm, SerialComm, TCPComm,
    BluetoothComm, BLEComm,
    RFCOMMComm, BluetoothScanner,
    is_bluetooth_port, bt_port_direction,
    scan_paired_bluetooth_devices, find_bluetooth_port_by_name,
)
from ._hand_exo import HandExo
from ._dual_hand_exo import DualHandExo
from ._fake_hand_exo import FakeHandExo
from ._lsl_client import LSLClient
from ._lsl_publisher import LSLMessagePublisher
from ._lsl_subscriber import LSLMarkerSubscriber, LSLNumericSubscriber
from ._gesture_controller import GestureController

__all__ = [
    # Communication backends
    "BaseComm", "SerialComm", "TCPComm",
    "BluetoothComm", "BLEComm",
    "RFCOMMComm", "BluetoothScanner",
    # Bluetooth helpers
    "is_bluetooth_port", "bt_port_direction",
    "scan_paired_bluetooth_devices", "find_bluetooth_port_by_name",
    # High-level device APIs
    "HandExo", "DualHandExo", "FakeHandExo",
    # LSL
    "LSLClient", "LSLMessagePublisher", "LSLMarkerSubscriber", "LSLNumericSubscriber",
    # Gesture controller
    "GestureController",
]
