import unittest

from nml_hand_exo.interface._hand_exo import HandExo


class GestureAngleApiTests(unittest.TestCase):
    def make_exo(self):
        exo = HandExo.__new__(HandExo)
        exo.commands = []
        exo._firmware_version = (0, 2, 16)
        exo.send_command = exo.commands.append
        return exo

    def test_formats_normalized_command(self):
        exo = self.make_exo()
        exo.set_gesture_angle("index", 25.0)
        self.assertEqual(exo.commands, ["set_gesture_angle:index:25"])

    def test_accepts_numeric_strings(self):
        exo = self.make_exo()
        exo.set_gesture_angle("wrist", "12.5")
        self.assertEqual(exo.commands, ["set_gesture_angle:wrist:12.5"])

    def test_normalizes_gesture_name(self):
        exo = self.make_exo()
        exo.set_gesture_angle(" Index ", 10)
        self.assertEqual(exo.commands, ["set_gesture_angle:index:10"])

    def test_accepts_individual_thumb_axes(self):
        exo = self.make_exo()
        exo.set_gesture_angle("thumbrot", 30)
        self.assertEqual(exo.commands, ["set_gesture_angle:thumbrot:30"])

    def test_rejects_non_numeric_percent(self):
        exo = self.make_exo()
        with self.assertRaises(ValueError):
            exo.set_gesture_angle("index", "closed")
        self.assertEqual(exo.commands, [])

    def test_rejects_whole_hand_and_non_finite_inputs(self):
        exo = self.make_exo()
        with self.assertRaises(ValueError):
            exo.set_gesture_angle("grasp", 50)
        with self.assertRaises(ValueError):
            exo.set_gesture_angle("index", float("nan"))
        self.assertEqual(exo.commands, [])

    def test_rejects_older_firmware(self):
        exo = self.make_exo()
        exo._firmware_version = (0, 2, 15)
        with self.assertRaises(RuntimeError):
            exo.set_gesture_angle("index", 50)
        self.assertEqual(exo.commands, [])


if __name__ == "__main__":
    unittest.main()
