import unittest

from nml_hand_exo.interface._gesture_protocol import (
    normalize_udp_gesture_angle_command,
)


class UDPGestureAngleCommandTests(unittest.TestCase):
    def test_non_angle_command_is_not_claimed(self):
        self.assertIsNone(normalize_udp_gesture_angle_command("set_gesture:grasp:flex"))

    def test_normalizes_gesture_and_numeric_value(self):
        command, gesture = normalize_udp_gesture_angle_command(
            "set_gesture_angle: Index : 12.50"
        )
        self.assertEqual(gesture, "index")
        self.assertEqual(command, "set_gesture_angle:index:12.5")

    def test_accepts_individual_thumb_axis(self):
        command, gesture = normalize_udp_gesture_angle_command(
            "set_gesture_angle:thumbrot:100"
        )
        self.assertEqual(gesture, "thumbrot")
        self.assertEqual(command, "set_gesture_angle:thumbrot:100")

    def test_rejects_unsafe_or_malformed_targets(self):
        for command in (
            "set_gesture_angle:grasp:50",
            "set_gesture_angle:index:-1",
            "set_gesture_angle:index:101",
            "set_gesture_angle:index:nan",
            "set_gesture_angle:index:50:extra",
        ):
            with self.subTest(command=command):
                with self.assertRaises(ValueError):
                    normalize_udp_gesture_angle_command(command)


if __name__ == "__main__":
    unittest.main()
