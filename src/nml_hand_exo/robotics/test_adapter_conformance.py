from __future__ import annotations

import unittest

from nml_hand_exo.robotics.bootstrap import register_builtin_robot_adapters
from nml_hand_exo.robotics.contracts import RobotActionPlan, RobotIntentType, RobotJointTarget
from nml_hand_exo.robotics.registry import GLOBAL_ROBOT_ADAPTER_REGISTRY


class AdapterConformanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        register_builtin_robot_adapters()
        cls._adapter_ids = ["nml_hand_exo", "virtual_openclaw"]

    def _create_adapter(self, adapter_id: str):
        if adapter_id == "nml_hand_exo":
            return GLOBAL_ROBOT_ADAPTER_REGISTRY.create("nml_hand_exo")
        return GLOBAL_ROBOT_ADAPTER_REGISTRY.create("virtual_openclaw")

    def test_all_adapters_pass_compatibility_handshake(self) -> None:
        for adapter_id in self._adapter_ids:
            adapter = self._create_adapter(adapter_id)
            try:
                report = adapter.compatibility_report()
                self.assertTrue(report.compatible, msg=f"{adapter_id} failed compatibility: {report.message}")
            finally:
                adapter.shutdown()

    def test_all_adapters_expose_minimum_capability_surface(self) -> None:
        for adapter_id in self._adapter_ids:
            adapter = self._create_adapter(adapter_id)
            try:
                caps = adapter.describe_capabilities()
                self.assertTrue(caps.joints, msg=f"{adapter_id} exposes no joints")
                self.assertTrue(caps.supported_intents, msg=f"{adapter_id} exposes no supported intents")
                self.assertGreaterEqual(caps.protocol_version, 1)
            finally:
                adapter.shutdown()

    def test_all_adapters_support_status_query_in_dry_run(self) -> None:
        for adapter_id in self._adapter_ids:
            adapter = self._create_adapter(adapter_id)
            try:
                plan = RobotActionPlan(intent_type=RobotIntentType.QUERY_STATUS, summary="status")
                result = adapter.execute_plan(plan, dry_run=True)
                self.assertTrue(result.success, msg=f"{adapter_id} failed status query")
            finally:
                adapter.shutdown()

    def test_all_adapters_support_single_joint_target_dry_run(self) -> None:
        for adapter_id in self._adapter_ids:
            adapter = self._create_adapter(adapter_id)
            try:
                plan = RobotActionPlan(
                    intent_type=RobotIntentType.SET_JOINT_TARGETS,
                    summary="move index",
                    joint_targets=[RobotJointTarget("index", 15.0)],
                    metadata={"auto_permission": "explicit_user_request"},
                )
                result = adapter.execute_plan(plan, dry_run=True)
                self.assertTrue(result.success, msg=f"{adapter_id} failed single-joint dry run")
            finally:
                adapter.shutdown()


if __name__ == "__main__":
    unittest.main()
