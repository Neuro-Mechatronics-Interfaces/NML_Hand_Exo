import struct
import unittest

from nml_hand_exo.interface._gesture_protocol import (
    CONTINUOUS_ACK_MAGIC,
    SET_FINGER_ANGLES_MAX,
    SET_FINGER_ANGLES_ORDER,
    clamp_finger_value,
    format_set_finger_angles,
    pack_continuous_ack,
    unpack_continuous_ack,
)


class SetFingerAnglesFormatTests(unittest.TestCase):
    def test_full_signed_array_in_wire_order(self):
        command = format_set_finger_angles(
            {
                "thumb": 100,
                "index": -100,
                "middle": 0,
                "ring": 50,
                "pinky": -50,
                "wrist": 25,
            }
        )
        self.assertEqual(command, "set_finger_angles:100:-100:0:50:-50:25")

    def test_wire_order_is_the_documented_order(self):
        self.assertEqual(
            SET_FINGER_ANGLES_ORDER,
            ("thumb", "index", "middle", "ring", "pinky", "wrist"),
        )

    def test_signed_zero_is_emitted_not_treated_as_hold(self):
        # 0 means "go to rest", which is a real command; only None/absent holds.
        command = format_set_finger_angles({"thumb": 0, "index": 0})
        self.assertEqual(command, "set_finger_angles:0:0")

    def test_missing_joint_becomes_empty_hold_field(self):
        command = format_set_finger_angles({"thumb": 80, "middle": 0, "pinky": -10})
        self.assertEqual(command, "set_finger_angles:80::0::-10")

    def test_none_holds_a_joint(self):
        command = format_set_finger_angles({"thumb": 80, "index": None, "middle": 0})
        self.assertEqual(command, "set_finger_angles:80::0")

    def test_trailing_held_joints_are_dropped(self):
        command = format_set_finger_angles({"thumb": 10, "index": -20})
        self.assertEqual(command, "set_finger_angles:10:-20")

    def test_leading_hold_is_kept(self):
        command = format_set_finger_angles({"index": -20})
        self.assertEqual(command, "set_finger_angles::-20")

    def test_float_values_are_rounded_to_int(self):
        # round() is banker's rounding: 12.5 -> 12, 13.5 -> 14. The command and
        # the ack both use round(), so they always agree on the integer.
        command = format_set_finger_angles({"thumb": 13.5, "index": -30.4})
        self.assertEqual(command, "set_finger_angles:14:-30")

    def test_rejects_empty_mapping(self):
        with self.assertRaises(ValueError):
            format_set_finger_angles({})

    def test_rejects_all_held(self):
        with self.assertRaises(ValueError):
            format_set_finger_angles({"thumb": None, "index": None})

    def test_rejects_out_of_range(self):
        for value in (-101, 100.6, 200):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    format_set_finger_angles({"index": value})

    def test_rejects_non_finite(self):
        for value in (float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    format_set_finger_angles({"index": value})

    def test_rejects_bool(self):
        with self.assertRaises(ValueError):
            format_set_finger_angles({"index": True})

    def test_rejects_non_numeric(self):
        with self.assertRaises(ValueError):
            format_set_finger_angles({"index": "abc"})


class ClampFingerValueTests(unittest.TestCase):
    def test_maps_unit_range_to_signed_hundred(self):
        self.assertEqual(clamp_finger_value(0.0 * 100), 0)
        self.assertEqual(clamp_finger_value(1.0 * 100), SET_FINGER_ANGLES_MAX)
        self.assertEqual(clamp_finger_value(-1.0 * 100), -SET_FINGER_ANGLES_MAX)

    def test_rounds_to_nearest_int(self):
        self.assertEqual(clamp_finger_value(53.4), 53)
        self.assertEqual(clamp_finger_value(-46.6), -47)

    def test_clamps_out_of_range(self):
        self.assertEqual(clamp_finger_value(250), SET_FINGER_ANGLES_MAX)
        self.assertEqual(clamp_finger_value(-250), -SET_FINGER_ANGLES_MAX)


class ContinuousAckTests(unittest.TestCase):
    def test_round_trips_sequence_and_signed_values(self):
        values = {
            "thumb": 100,
            "index": -100,
            "middle": 0,
            "ring": 50,
            "pinky": -50,
            "wrist": 25,
        }
        frame = pack_continuous_ack(1234567, SET_FINGER_ANGLES_ORDER, values)
        self.assertTrue(frame.startswith(CONTINUOUS_ACK_MAGIC))
        seq, decoded = unpack_continuous_ack(frame)
        self.assertEqual(seq, 1234567)
        self.assertEqual(decoded, values)

    def test_missing_joint_acked_as_zero(self):
        frame = pack_continuous_ack(7, SET_FINGER_ANGLES_ORDER, {"thumb": 40})
        _, decoded = unpack_continuous_ack(frame)
        self.assertEqual(decoded["thumb"], 40)
        self.assertEqual(decoded["index"], 0)

    def test_sequence_wraps_uint32(self):
        frame = pack_continuous_ack(2**32 + 5, SET_FINGER_ANGLES_ORDER, {"thumb": 0})
        seq, _ = unpack_continuous_ack(frame)
        self.assertEqual(seq, 5)

    def test_rejects_wrong_magic(self):
        self.assertIsNone(unpack_continuous_ack(b"XXXX" + b"\x00" * 12))

    def test_rejects_truncated_frame(self):
        frame = pack_continuous_ack(1, SET_FINGER_ANGLES_ORDER, {"thumb": 0})
        self.assertIsNone(unpack_continuous_ack(frame[:-2]))

    def test_rejects_joint_count_mismatch(self):
        frame = pack_continuous_ack(1, SET_FINGER_ANGLES_ORDER, {"thumb": 0})
        self.assertIsNone(unpack_continuous_ack(frame, joints=("thumb", "index")))


if __name__ == "__main__":
    unittest.main()
