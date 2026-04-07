from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from nml_hand_exo.robotics.policy_profiles import apply_policy_profile_defaults, load_policy_profile_manifest


class PolicyProfileTests(unittest.TestCase):
    def test_load_policy_profile_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            profile_path = Path(tmp_dir) / "safe.yaml"
            profile_path.write_text(
                "profile_version: 1\n"
                "profile_id: safe\n"
                "defaults:\n"
                "  confirmation_mode: strict\n"
                "  max_relative_angle: 90.0\n",
                encoding="utf-8",
            )

            manifest, resolved = load_policy_profile_manifest("", str(profile_path))
            self.assertEqual("safe", manifest.profile_id)
            self.assertEqual("strict", manifest.defaults["confirmation_mode"])
            self.assertTrue(resolved.exists())

    def test_apply_policy_profile_defaults(self) -> None:
        args = SimpleNamespace(confirmation_mode="relaxed", max_relative_angle=180.0)
        parser_defaults = {"confirmation_mode": "relaxed", "max_relative_angle": 180.0}

        applied = apply_policy_profile_defaults(
            args,
            {"confirmation_mode": "balanced", "max_relative_angle": 120.0},
            parser_defaults,
        )

        self.assertEqual(["confirmation_mode", "max_relative_angle"], applied)
        self.assertEqual("balanced", args.confirmation_mode)
        self.assertEqual(120.0, args.max_relative_angle)


if __name__ == "__main__":
    unittest.main()
