import unittest

from nml_hand_exo.interface._hand_exo import (
    ANGLE_ADDRESSABLE_GESTURES,
    FW_PER_JOINT_REST,
    FW_RAD_GESTURE,
    GESTURE_MIN_FIRMWARE,
    HandExo,
    parse_firmware_version,
)


class _RecordingComm:
    """Minimal BaseComm stand-in that records writes and replays canned replies."""

    def __init__(self, version_reply="Version: 0.3.0"):
        self.verbose = False
        self.sent = []
        self.version_reply = version_reply

    def connect(self):
        pass

    def send(self, message):
        self.sent.append(message)

    # HandExo.send_command / _receive route through these names.
    def write(self, message):
        self.sent.append(message)

    def read(self, *args, **kwargs):
        return self.version_reply


def _exo(version_reply="Version: 0.3.0"):
    exo = HandExo(_RecordingComm(version_reply), auto_connect=False)
    # Skip the serial round-trip; feature gates only consume the parsed tuple.
    exo._firmware_version = parse_firmware_version(version_reply)
    return exo


class ParseFirmwareVersionTests(unittest.TestCase):
    def test_parses_bare_and_decorated_forms(self):
        self.assertEqual(parse_firmware_version("0.3.0"), (0, 3, 0))
        self.assertEqual(parse_firmware_version("Version: 0.3.0;"), (0, 3, 0))
        self.assertEqual(parse_firmware_version(" 0.2.17 "), (0, 2, 17))

    def test_unparseable_input_sorts_below_every_real_version(self):
        for bad in ("", None, "unknown", "vNext"):
            self.assertEqual(parse_firmware_version(bad), ())
            self.assertLess(parse_firmware_version(bad), (0, 0, 1))

    def test_numeric_not_lexical_ordering(self):
        # "0.2.17" must not sort above "0.3.0" the way string compare would.
        self.assertLess(parse_firmware_version("0.2.17"), parse_firmware_version("0.3.0"))
        self.assertGreaterEqual(parse_firmware_version("0.3.0"), FW_PER_JOINT_REST)
        self.assertLess(parse_firmware_version("0.2.17"), FW_PER_JOINT_REST)


class FeatureGateTests(unittest.TestCase):
    def test_new_firmware_allows_rest_wrist_and_angle(self):
        exo = _exo("0.3.0")
        self.assertTrue(exo.firmware_at_least(FW_PER_JOINT_REST))
        exo.set_gesture("index", "rest")
        exo.set_gesture("wrist", "flex")
        exo.set_gesture_angle("index", 50)
        self.assertEqual(
            exo.device.sent[-3:],
            [
                "set_gesture:index:rest\n",
                "set_gesture:wrist:flex\n",
                "set_gesture_angle:index:50\n",
            ],
        )

    def test_old_firmware_raises_instead_of_silently_acking(self):
        exo = _exo("0.2.17")
        self.assertFalse(exo.firmware_at_least(FW_PER_JOINT_REST))
        for call in (
            lambda: exo.set_gesture("index", "rest"),
            lambda: exo.set_gesture("wrist", "flex"),
            lambda: exo.set_gesture_angle("index", 50),
        ):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual(exo.device.sent, [], "no command should reach old firmware")

    def test_unknown_firmware_fails_closed(self):
        exo = _exo("garbage")
        with self.assertRaises(RuntimeError):
            exo.set_gesture_angle("index", 50)

    def test_flex_and_extend_are_not_gated(self):
        exo = _exo("0.2.17")
        exo.set_gesture("index", "flex")
        exo.set_gesture("index", "extend")
        self.assertEqual(len(exo.device.sent), 2)

    def test_non_numeric_percent_rejected(self):
        exo = _exo("0.3.0")
        with self.assertRaises(ValueError):
            exo.set_gesture_angle("index", "halfway")

    def test_out_of_range_percent_is_sent_for_firmware_to_clamp(self):
        exo = _exo("0.3.0")
        exo.set_gesture_angle("index", 150)
        self.assertEqual(exo.device.sent[-1], "set_gesture_angle:index:150\n")

    def test_angle_addressable_set_matches_firmware_gesture_library(self):
        self.assertEqual(
            set(ANGLE_ADDRESSABLE_GESTURES),
            {"thumb", "index", "middle", "ring", "pinky", "wrist", "rad"},
        )


class RadGestureGateTests(unittest.TestCase):
    """`rad` landed one version after `wrist`, so 0.3.0 must not admit it."""

    def test_rad_requires_a_later_version_than_wrist(self):
        self.assertGreater(FW_RAD_GESTURE, FW_PER_JOINT_REST)
        self.assertEqual(GESTURE_MIN_FIRMWARE["rad"], FW_RAD_GESTURE)
        self.assertEqual(GESTURE_MIN_FIRMWARE["wrist"], FW_PER_JOINT_REST)

    def test_rad_rejected_on_the_version_that_only_has_wrist(self):
        exo = _exo("0.3.0")
        exo.set_gesture("wrist", "flex")          # allowed at 0.3.0
        exo.set_gesture_angle("wrist", 30)
        for call in (
            lambda: exo.set_gesture("rad", "flex"),
            lambda: exo.set_gesture_angle("rad", 25),
        ):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual(len(exo.device.sent), 2, "only the wrist calls go out")

    def test_rad_allowed_on_0_3_1(self):
        exo = _exo("0.3.1")
        exo.set_gesture("rad", "rest")
        exo.set_gesture_angle("rad", 25)
        self.assertEqual(
            exo.device.sent,
            ["set_gesture:rad:rest\n", "set_gesture_angle:rad:25\n"],
        )

    def test_digits_are_not_gated_by_the_gesture_map(self):
        for digit in ("thumb", "index", "middle", "ring", "pinky"):
            self.assertNotIn(digit, GESTURE_MIN_FIRMWARE)


if __name__ == "__main__":
    unittest.main()
