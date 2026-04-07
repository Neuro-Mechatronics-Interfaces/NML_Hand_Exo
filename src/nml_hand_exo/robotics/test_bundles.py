from __future__ import annotations

import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from nml_hand_exo.robotics.bundles import apply_bundle_defaults, load_bundle_manifest


class DeploymentBundleTests(unittest.TestCase):
    def test_load_bundle_manifest_from_explicit_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bundle_path = root / "bundle.yaml"
            (root / "robot.yaml").write_text("config_version: 1\nadapter_id: nml_hand_exo\n", encoding="utf-8")
            bundle_path.write_text(
                "bundle_version: 1\n"
                "bundle_id: test_bundle\n"
                "adapter:\n"
                "  id: nml_hand_exo\n"
                "  config: ./robot.yaml\n"
                "runtime_defaults:\n"
                "  dry_run: true\n",
                encoding="utf-8",
            )

            manifest, resolved = load_bundle_manifest("", str(bundle_path))
            self.assertEqual("test_bundle", manifest.bundle_id)
            self.assertEqual("nml_hand_exo", manifest.adapter_id)
            self.assertTrue(manifest.adapter_config_path.endswith("robot.yaml"))
            self.assertTrue(resolved.exists())

    def test_load_bundle_manifest_with_policy_skill_packs_and_telemetry(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            (root / "robot.yaml").write_text("config_version: 1\nadapter_id: nml_hand_exo\n", encoding="utf-8")
            (root / "policy.yaml").write_text("profile_version: 1\nprofile_id: safe\n", encoding="utf-8")
            (root / "skill.md").write_text("# skill\n", encoding="utf-8")
            bundle_path = root / "bundle.yaml"
            bundle_path.write_text(
                "bundle_version: 1\n"
                "bundle_id: test_bundle\n"
                "adapter:\n"
                "  id: nml_hand_exo\n"
                "  config: ./robot.yaml\n"
                "policy_profile: ./policy.yaml\n"
                "skill_packs:\n"
                "  - ./skill.md\n"
                "telemetry:\n"
                "  schema_version: v1\n"
                "  strict: true\n",
                encoding="utf-8",
            )

            manifest, _ = load_bundle_manifest("", str(bundle_path))
            self.assertTrue(manifest.policy_profile_path.endswith("policy.yaml"))
            self.assertEqual(1, len(manifest.skill_pack_paths))
            self.assertTrue(manifest.skill_pack_paths[0].endswith("skill.md"))
            self.assertEqual("v1", manifest.telemetry_schema_version)
            self.assertTrue(manifest.telemetry_schema_strict)

    def test_load_bundle_manifest_rejects_required_signature_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bundle_path = root / "bundle.yaml"
            bundle_path.write_text(
                "bundle_version: 1\n"
                "bundle_id: sig_bundle\n"
                "adapter:\n"
                "  id: nml_hand_exo\n"
                "signature:\n"
                "  required: true\n"
                "  sha256: deadbeef\n",
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_bundle_manifest("", str(bundle_path))

    def test_load_bundle_manifest_accepts_required_signature_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            bundle_path = root / "bundle.yaml"
            bundle_path.write_text(
                "bundle_version: 1\n"
                "bundle_id: sig_bundle\n"
                "adapter:\n"
                "  id: nml_hand_exo\n"
                "signature:\n"
                "  required: true\n"
                "  file: ./bundle.sha256\n",
                encoding="utf-8",
            )
            digest = hashlib.sha256(bundle_path.read_bytes()).hexdigest().lower()
            (root / "bundle.sha256").write_text(f"{digest}  bundle.yaml\n", encoding="utf-8")

            manifest, _ = load_bundle_manifest("", str(bundle_path))
            self.assertTrue(manifest.signature_required)
            self.assertTrue(manifest.signature_verified)

    def test_apply_bundle_defaults_respects_parser_defaults(self) -> None:
        args = SimpleNamespace(dry_run=False, provider="heuristic")
        parser_defaults = {"dry_run": False, "provider": "heuristic"}
        applied = apply_bundle_defaults(args, {"dry_run": True, "provider": "groq"}, parser_defaults)
        self.assertEqual(["dry_run", "provider"], applied)
        self.assertTrue(args.dry_run)
        self.assertEqual("groq", args.provider)

    def test_load_bundle_manifest_rejects_unknown_root_keys(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            bundle_path = Path(tmp_dir) / "bundle.yaml"
            bundle_path.write_text(
                "bundle_version: 1\n"
                "adapter:\n"
                "  id: nml_hand_exo\n"
                "bad_key: true\n",
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_bundle_manifest("", str(bundle_path))


if __name__ == "__main__":
    unittest.main()
