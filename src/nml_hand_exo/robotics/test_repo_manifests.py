from __future__ import annotations

import unittest
from pathlib import Path

from nml_hand_exo.robotics.bundles import list_available_bundles, load_bundle_manifest
from nml_hand_exo.robotics.config import load_robot_adapter_config
from nml_hand_exo.robotics.policy_profiles import list_available_policy_profiles, load_policy_profile_manifest


class RepositoryManifestValidationTests(unittest.TestCase):
    def test_all_repo_bundles_load_and_reference_valid_configs(self) -> None:
        bundle_ids = list_available_bundles()
        self.assertTrue(bundle_ids, msg="No deployment bundles found in config/bundles")

        for bundle_id in bundle_ids:
            manifest, _ = load_bundle_manifest(bundle_id)
            self.assertTrue(manifest.adapter_id)

            cfg, cfg_path = load_robot_adapter_config(
                manifest.adapter_id,
                manifest.adapter_config_path or None,
            )
            self.assertEqual(manifest.adapter_id, cfg.adapter_id)
            self.assertIsNotNone(cfg_path)

            if manifest.policy_profile_path:
                profile, profile_path = load_policy_profile_manifest("", manifest.policy_profile_path)
                self.assertTrue(profile.profile_id)
                self.assertTrue(profile_path.exists())

            for skill_pack_path in manifest.skill_pack_paths:
                self.assertTrue(Path(skill_pack_path).exists(), msg=f"Missing skill pack: {skill_pack_path}")

    def test_all_repo_policy_profiles_load(self) -> None:
        profile_ids = list_available_policy_profiles()
        self.assertTrue(profile_ids, msg="No policy profiles found in config/policy_profiles")

        for profile_id in profile_ids:
            profile, profile_path = load_policy_profile_manifest(profile_id)
            self.assertEqual(profile_id, profile.profile_id)
            self.assertTrue(profile_path.exists())


if __name__ == "__main__":
    unittest.main()
