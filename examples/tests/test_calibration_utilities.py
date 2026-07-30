import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from nml_hand_exo.calibration import (
    CalibrationProfileStore,
    build_motor_orientation,
    determine_run_number,
    normalize_angle,
)
from nml_hand_exo.interface._serial_ports import format_port_label


class CalibrationProfileStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.profiles_dir = Path(self.temp_dir.name) / "profiles"
        self.store = CalibrationProfileStore(self.profiles_dir)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_saves_loads_lists_and_filters_profiles(self):
        self.store.save_profile("right_user", {"motors": {}}, side="right")
        self.store.save_profile("left_user", {"motors": {}}, side="left")

        self.assertEqual(self.store.list_profiles(), ["left_user", "right_user"])
        self.assertEqual(self.store.list_profiles("left"), ["left_user"])
        self.assertEqual(self.store.list_profiles("right"), ["right_user"])
        self.assertEqual(self.store.load_profile("left_user")["side"], "left")

    def test_save_side_argument_overrides_stale_payload_metadata(self):
        self.store.save_profile("participant", {"side": "right"}, side="left")

        self.assertEqual(self.store.load_profile("participant")["side"], "left")

    def test_defaults_are_side_specific_with_right_legacy_fallback_only(self):
        self.profiles_dir.mkdir(parents=True)
        self.store.config_path.write_text(
            json.dumps({"default": "legacy_right"}), encoding="utf-8"
        )

        self.assertEqual(
            self.store.get_default_profile_name("right"), "legacy_right"
        )
        self.assertIsNone(self.store.get_default_profile_name("left"))

        self.store.set_default_profile("left_user", "left")
        self.store.set_default_profile("right_user", "right")
        self.assertEqual(self.store.get_default_profile_name("left"), "left_user")
        self.assertEqual(
            self.store.get_default_profile_name("right"), "right_user"
        )

    def test_profile_names_cannot_escape_the_store_directory(self):
        for name in ("", "..", "nested/profile", "nested\\profile", "config"):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    self.store.profile_path(name)


class ROMUtilityTests(unittest.TestCase):
    def test_angle_normalization_and_orientation_defaults(self):
        self.assertEqual(normalize_angle(12.0, 10.0, False), 2.0)
        self.assertEqual(normalize_angle(8.0, 10.0, True), 2.0)
        orientation = build_motor_orientation(
            {"motors": {"index": {"home": 42, "flip": True}}},
            ["index", "middle"],
        )
        self.assertEqual(orientation["index"], {"home": 42.0, "flip": True})
        self.assertEqual(orientation["middle"], {"home": 0.0, "flip": False})

    def test_run_number_ignores_unrelated_and_malformed_files(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            for name in (
                "p01_rom_20260724_1.csv",
                "p01_rom_20260724_3.csv",
                "p01_rom_20260724_bad.csv",
                "other_rom_20260724_20.csv",
                "p01_rom_20260724_99.txt",
            ):
                (output_dir / name).touch()

            self.assertEqual(
                determine_run_number("p01", "20260724", output_dir), 4
            )


class SerialPortUtilityTests(unittest.TestCase):
    def test_formats_metadata_and_connection_tags(self):
        port = SimpleNamespace(
            device="COM7",
            description="NML_EXO USB Serial Device",
            hwid="USB VID:PID=1234:ABCD",
            manufacturer="NML",
            serial_number="xyz",
            vid=0x1234,
            pid=0xABCD,
        )

        label = format_port_label(port)

        self.assertIn("COM7", label)
        self.assertIn("[USB, NML_EXO]", label)
        self.assertIn("VID:1234 PID:ABCD", label)
        self.assertIn("SN:xyz", label)


if __name__ == "__main__":
    unittest.main()
