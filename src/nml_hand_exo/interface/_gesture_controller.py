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

    def set_gesture(self, gesture: str, state: str) -> None:
        """
        Set the gesture and state on the exoskeleton.

        Args:
            gesture: The gesture name (e.g., "IndexPinch", "Hand", etc.).
            state: The state ("Open" or "Close").
        """
        if state not in ["open", "close"]:
            raise ValueError(f"Invalid state: {state}. Must be 'Open' or 'Close'.")

        if self.verbose:
            print(f"[GestureController] Setting gesture '{gesture}' to state '{state}'")

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