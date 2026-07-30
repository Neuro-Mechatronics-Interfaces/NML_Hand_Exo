import unittest

from nml_hand_exo.interface._hand_exo import (
    ANGLE_ADDRESSABLE_GESTURES,
    FW_CURRENT_BUDGET,
    FW_GESTURE_ANGLE_READBACK,
    FW_PER_JOINT_REST,
    FW_RAD_GESTURE,
    GESTURE_ANGLE_ABOVE_RANGE,
    GESTURE_ANGLE_BELOW_RANGE,
    GESTURE_ANGLE_PREFIX,
    GESTURE_ANGLE_UNAVAILABLE,
    GESTURE_MAX_FIRMWARE,
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
        #: Reply handed to the next _receive(); tests set this for read-back APIs.
        self.next_reply = None

    def connect(self):
        pass

    def send(self, message):
        self.sent.append(message)

    # HandExo.send_command / _receive route through these names.
    def write(self, message):
        self.sent.append(message)

    def receive(self, wait_until_return=False, timeout=None):
        if self.next_reply is not None:
            reply, self.next_reply = self.next_reply, None
            return reply
        return self.version_reply

    def read(self, *args, **kwargs):
        return self.receive()


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
            {"thumb", "index", "middle", "ring", "pinky", "wrist"},
        )


class RadGestureGateTests(unittest.TestCase):
    """`rad` exists only between 0.3.1 and 0.6.0 -- gated on both sides."""

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

    def test_rad_allowed_on_the_versions_that_have_it(self):
        for version in ("0.3.1", "0.5.0"):
            exo = _exo(version)
            exo.set_gesture("rad", "rest")
            exo.set_gesture_angle("rad", 25)
            self.assertEqual(
                exo.device.sent,
                ["set_gesture:rad:rest\n", "set_gesture_angle:rad:25\n"],
                version,
            )

    def test_rad_rejected_again_once_wrist2_folded_into_wrist(self):
        # Firmware ACKs an unknown gesture, so an ungated call would look like
        # it worked while wrist2 never moved.
        exo = _exo("0.6.0")
        self.assertEqual(GESTURE_MAX_FIRMWARE["rad"], FW_GESTURE_ANGLE_READBACK)
        for call in (
            lambda: exo.set_gesture("rad", "flex"),
            lambda: exo.set_gesture_angle("rad", 25),
            lambda: exo.get_gesture_angle("rad"),
        ):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual(exo.device.sent, [])
        # ...and `wrist` still works, since it is what absorbed the axis.
        exo.set_gesture_angle("wrist", 25)
        self.assertEqual(exo.device.sent, ["set_gesture_angle:wrist:25\n"])

    def test_removed_gestures_are_not_offered_as_addressable(self):
        self.assertNotIn("rad", ANGLE_ADDRESSABLE_GESTURES)

    def test_digits_are_not_gated_by_the_gesture_map(self):
        for digit in ("thumb", "index", "middle", "ring", "pinky"):
            self.assertNotIn(digit, GESTURE_MIN_FIRMWARE)
            self.assertNotIn(digit, GESTURE_MAX_FIRMWARE)


class CurrentBudgetTests(unittest.TestCase):
    """The combined-current budget landed in 0.4.0."""

    _STATUS = "\n".join([
        "Current budget:",
        "  total_budget_mA: 800",
        "  hold_current_mA: 25",
        "  governor: on",
        "  measured_total_mA: 512",
        "  scale: 0.640",
        "  measurement_trusted: true",
        "Motor 0: {name: wrist, id: 11, nominal_mA: 200, applied_mA: 128, state: moving}",
        "Motor 1: {name: index, id: 16, nominal_mA: 200, applied_mA: 25, state: hold}",
    ])

    def test_gated_off_on_the_gesture_only_versions(self):
        exo = _exo("0.3.1")
        for call in (
            lambda: exo.set_total_current_limit(800),
            lambda: exo.get_total_current_limit(),
            lambda: exo.set_hold_current(25),
            lambda: exo.set_current_governor(True),
            lambda: exo.current_status(),
        ):
            with self.assertRaises(RuntimeError):
                call()
        self.assertEqual(exo.device.sent, [])

    def test_commands_sent_on_0_4_0(self):
        exo = _exo("0.4.0")
        exo.set_total_current_limit(1200)
        exo.set_hold_current(30)
        exo.set_current_governor(False)
        self.assertEqual(
            exo.device.sent,
            [
                "set_total_current_lim:1200\n",
                "set_hold_current:30\n",
                "set_current_governor:off\n",
            ],
        )

    def test_rejects_nonsense_budgets(self):
        exo = _exo("0.4.0")
        for bad in (0, -5):
            with self.assertRaises(ValueError):
                exo.set_total_current_limit(bad)
        with self.assertRaises(ValueError):
            exo.set_total_current_limit("lots")
        with self.assertRaises(ValueError):
            exo.set_hold_current(-1)
        self.assertEqual(exo.device.sent, [])

    def test_current_status_parses_budget_and_per_motor_allocation(self):
        exo = _exo("0.4.0")
        exo.device.next_reply = self._STATUS
        status = exo.current_status()
        self.assertEqual(status["total_budget_mA"], 800)
        self.assertEqual(status["hold_current_mA"], 25)
        self.assertEqual(status["measured_total_mA"], 512)
        self.assertAlmostEqual(status["scale"], 0.640)
        self.assertIs(status["governor"], True)
        self.assertIs(status["measurement_trusted"], True)
        self.assertEqual(
            status["motors"][11],
            {"nominal_mA": 200, "applied_mA": 128, "state": "moving"},
        )
        self.assertEqual(status["motors"][16]["state"], "hold")
        # applied < nominal is the signal that the budget is actively clamping.
        self.assertLess(
            status["motors"][11]["applied_mA"], status["motors"][11]["nominal_mA"]
        )

    def test_unsampled_aggregate_reads_as_none_not_zero(self):
        exo = _exo("0.4.0")
        exo.device.next_reply = self._STATUS.replace(
            "measured_total_mA: 512", "measured_total_mA: n/a"
        )
        status = exo.current_status()
        self.assertIsNone(status["measured_total_mA"])

    def test_budget_readback_survives_firmware_clamping(self):
        exo = _exo("0.4.0")
        exo.device.next_reply = "Total current limit: 450 mA"
        self.assertEqual(exo.get_total_current_limit(), 450.0)


class GestureAngleReadbackTests(unittest.TestCase):
    """`get_gesture_angle` and the re-anchored gesture axis landed in 0.6.0."""

    REPLY = (GESTURE_ANGLE_PREFIX + " thumb=12 index=0 middle=45 ring=101 "
             "pinky=100 wrist=255;")

    def test_gated_off_on_every_earlier_version(self):
        for version in ("0.3.0", "0.3.1", "0.5.0"):
            exo = _exo(version)
            with self.assertRaises(RuntimeError, msg=version):
                exo.get_gesture_angle()
            self.assertEqual(exo.device.sent, [], version)

    def test_readback_lands_after_the_current_budget(self):
        self.assertGreater(FW_GESTURE_ANGLE_READBACK, FW_CURRENT_BUDGET)

    def test_reads_every_joint_including_the_status_codes(self):
        exo = _exo("0.6.0")
        exo.device.next_reply = self.REPLY
        angles = exo.get_gesture_angle()
        self.assertEqual(exo.device.sent[-1], "get_gesture_angle:all\n")
        self.assertEqual(angles["index"], 0)
        self.assertEqual(angles["thumb"], 12)
        # Codes are distinguishable from real percentages: 0-100 is a position,
        # anything above it is a status.
        self.assertEqual(angles["ring"], GESTURE_ANGLE_BELOW_RANGE)
        self.assertEqual(angles["wrist"], GESTURE_ANGLE_UNAVAILABLE)
        self.assertGreater(GESTURE_ANGLE_BELOW_RANGE, 100)
        self.assertGreater(GESTURE_ANGLE_ABOVE_RANGE, GESTURE_ANGLE_BELOW_RANGE)

    def test_single_gesture_query_is_addressed_by_name(self):
        exo = _exo("0.6.0")
        exo.device.next_reply = GESTURE_ANGLE_PREFIX + " wrist=33;"
        self.assertEqual(exo.get_gesture_angle("Wrist"), {"wrist": 33})
        self.assertEqual(exo.device.sent[-1], "get_gesture_angle:wrist\n")

    def test_covers_exactly_the_angle_addressable_gestures(self):
        exo = _exo("0.6.0")
        exo.device.next_reply = self.REPLY
        self.assertEqual(
            set(exo.get_gesture_angle()), set(ANGLE_ADDRESSABLE_GESTURES)
        )

    def test_silent_device_reads_as_empty_rather_than_raising(self):
        exo = _exo("0.6.0")
        exo.device.next_reply = ""
        self.assertEqual(exo.get_gesture_angle(), {})


if __name__ == "__main__":
    unittest.main()
