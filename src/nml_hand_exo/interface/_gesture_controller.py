from ._hand_exo import HandExo
from ._fake_hand_exo import FakeHandExo


class GestureController:
    """
    A simple gesture controller for the Hand Exoskeleton.

    It allows setting gestures and states (open/close) on the exoskeleton.

    Args:
        exo: An instance of HandExo or FakeHandExo.
        verbose: If True, prints verbose output.
    """

    def __init__(self, exo: HandExo | FakeHandExo, verbose: bool = False):
        self.exo = exo
        self.verbose = verbose
        self.current_gesture: str = "Rest"
        self.current_state: str = "Open"

    #: Multi-joint postures (grasp, keygrip, pinch_*, peace) use these.
    POSTURE_STATES = ("open", "close")

    #: Per-joint gestures (thumb/index/middle/ring/pinky/wrist) use these.
    #: ``rest`` needs firmware >= 0.3.0; HandExo.set_gesture enforces that.
    JOINT_STATES = ("extend", "rest", "flex")

    def set_gesture(self, gesture: str, state: str) -> None:
        """
        Set the gesture and state on the exoskeleton.

        Args:
            gesture: The gesture name (e.g., "pinch_index", "index", "wrist").
            state: "open"/"close" for postures, or "extend"/"rest"/"flex" for
                per-joint gestures.

        Raises:
            ValueError: If the state is not one of the accepted values.
            RuntimeError: If the state needs newer firmware than is connected.
        """
        valid = self.POSTURE_STATES + self.JOINT_STATES
        if state not in valid:
            raise ValueError(
                f"Invalid state: {state}. Must be one of {', '.join(valid)}."
            )

        if self.verbose:
            print(f"[GestureController] Setting gesture '{gesture}' to state '{state}'")

        # Prefer HandExo.set_gesture so its firmware feature gate applies; fall
        # back to the raw command for exo objects that do not implement it.
        if hasattr(self.exo, "set_gesture"):
            self.exo.set_gesture(gesture, state)
        else:
            self.exo.send_command(f"set_gesture:{gesture}:{state}")

        self.current_gesture = gesture
        self.current_state = state

    def get_current_gesture(self) -> tuple[str, str]:
        """
        Get the current gesture and state.

        Returns:
            A tuple of (gesture, state).
        """
        return self.current_gesture, self.current_state