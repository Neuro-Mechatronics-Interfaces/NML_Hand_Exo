from ._interfaces import BaseComm, SerialComm, TCPComm, DualSerialComm
from ._hand_exo import HandExo
from ._dual_hand_exo import DualHandExo
from ._fake_hand_exo import FakeHandExo
from ._gesture_controller import GestureController

# LSL dependencies (pylsl/liblsl) are optional in CI and non-streaming installs.
try:
	from ._lsl_client import LSLClient
	from ._lsl_publisher import LSLMessagePublisher
	from ._lsl_subscriber import LSLMarkerSubscriber, LSLNumericSubscriber
except Exception:
	LSLClient = None
	LSLMessagePublisher = None
	LSLMarkerSubscriber = None
	LSLNumericSubscriber = None
