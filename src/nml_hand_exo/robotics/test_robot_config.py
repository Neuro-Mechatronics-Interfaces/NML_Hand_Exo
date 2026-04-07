from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from nml_hand_exo.robotics.config import apply_cli_defaults_from_config, load_robot_adapter_config


class RobotConfigTests(unittest.TestCase):
    def test_load_robot_adapter_config_from_explicit_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg_path = Path(tmp_dir) / "adapter.yaml"
            cfg_path.write_text(
                "adapter_id: virtual_openclaw\n"
                "adapter_kwargs:\n"
                "  device:\n"
                "    name: Test Virtual\n"
                "cli_defaults:\n"
                "  dry_run: true\n"
                "  assistant_tone: neutral\n",
                encoding="utf-8",
            )

            cfg, resolved = load_robot_adapter_config("virtual_openclaw", str(cfg_path))
            self.assertIsNotNone(resolved)
            self.assertEqual("virtual_openclaw", cfg.adapter_id)
            self.assertEqual("Test Virtual", cfg.adapter_kwargs["device"]["name"])
            self.assertEqual("neutral", cfg.cli_defaults["assistant_tone"])

    def test_apply_cli_defaults_only_when_argument_is_default(self) -> None:
        args = SimpleNamespace(dry_run=False, assistant_tone="warm")
        parser_defaults = {"dry_run": False, "assistant_tone": "warm"}
        applied = apply_cli_defaults_from_config(
            args,
            {"dry_run": True, "assistant_tone": "neutral"},
            parser_defaults,
        )
        self.assertEqual(["dry_run", "assistant_tone"], applied)
        self.assertTrue(args.dry_run)
        self.assertEqual("neutral", args.assistant_tone)

    def test_apply_cli_defaults_does_not_override_non_default_values(self) -> None:
        args = SimpleNamespace(dry_run=True, assistant_tone="concise")
        parser_defaults = {"dry_run": False, "assistant_tone": "warm"}
        applied = apply_cli_defaults_from_config(
            args,
            {"dry_run": False, "assistant_tone": "neutral"},
            parser_defaults,
        )
        self.assertEqual([], applied)
        self.assertTrue(args.dry_run)
        self.assertEqual("concise", args.assistant_tone)

    def test_load_robot_adapter_config_rejects_unknown_root_keys(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg_path = Path(tmp_dir) / "adapter.yaml"
            cfg_path.write_text(
                "adapter_id: nml_hand_exo\n"
                "config_version: 1\n"
                "unknown_key: value\n",
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_robot_adapter_config("nml_hand_exo", str(cfg_path))

    def test_load_robot_adapter_config_rejects_unsupported_config_version(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg_path = Path(tmp_dir) / "adapter.yaml"
            cfg_path.write_text(
                "adapter_id: nml_hand_exo\n"
                "config_version: 99\n",
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_robot_adapter_config("nml_hand_exo", str(cfg_path))


if __name__ == "__main__":
    unittest.main()
