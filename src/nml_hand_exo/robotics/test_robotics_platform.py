from __future__ import annotations

import unittest

from nml_hand_exo.robotics.adapters import NmlHandExoRobotAdapter
from nml_hand_exo.robotics.bootstrap import register_builtin_robot_adapters
from nml_hand_exo.robotics.contracts import RobotActionPlan, RobotIntentType, RobotJointTarget
from nml_hand_exo.robotics.policy import RobotSafetyPolicyEngine
from nml_hand_exo.robotics.registry import GLOBAL_ROBOT_ADAPTER_REGISTRY


class RoboticsPlatformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from nml_hand_exo.robotics.ai_runtime import get_ai_runtime

        try:
            ai_runtime = get_ai_runtime()
        except ModuleNotFoundError as exc:
            raise unittest.SkipTest(f"NeuroBridge runtime unavailable for robotics tests: {exc}")

        cls.SimulatedHandExo = ai_runtime.SimulatedHandExo

    def test_registry_bootstrap_registers_nml_adapter(self) -> None:
        register_builtin_robot_adapters()
        self.assertTrue(GLOBAL_ROBOT_ADAPTER_REGISTRY.has("nml_hand_exo"))
        adapter = GLOBAL_ROBOT_ADAPTER_REGISTRY.create("nml_hand_exo")
        self.assertEqual("nml_hand_exo", adapter.adapter_id)
        self.assertTrue(adapter.health_check().healthy)
        adapter.shutdown()

    def test_registry_bootstrap_registers_virtual_openclaw_adapter(self) -> None:
        register_builtin_robot_adapters()
        self.assertTrue(GLOBAL_ROBOT_ADAPTER_REGISTRY.has("virtual_openclaw"))
        adapter = GLOBAL_ROBOT_ADAPTER_REGISTRY.create("virtual_openclaw")
        self.assertEqual("virtual_openclaw", adapter.adapter_id)
        self.assertTrue(adapter.health_check().healthy)
        adapter.shutdown()

    def test_policy_engine_blocks_unsupported_joint(self) -> None:
        adapter = NmlHandExoRobotAdapter(exo=self.SimulatedHandExo())
        plan = RobotActionPlan(
            intent_type=RobotIntentType.SET_JOINT_TARGETS,
            summary="Move invalid joint",
            joint_targets=[RobotJointTarget("elbow", 10.0)],
        )
        policy = RobotSafetyPolicyEngine()
        decision = policy.evaluate(
            plan,
            capabilities=adapter.describe_capabilities(),
            state=adapter.collect_state(),
        )
        self.assertFalse(decision.allow)

    def test_nml_adapter_dry_run_executes_plan_contract(self) -> None:
        exo = self.SimulatedHandExo()
        exo.connect()
        adapter = NmlHandExoRobotAdapter(exo=exo)
        plan = RobotActionPlan(
            intent_type=RobotIntentType.SET_JOINT_TARGETS,
            summary="Close pinky in dry run",
            joint_targets=[RobotJointTarget("pinky", 30.0)],
            ask_for_confirmation=False,
            metadata={"auto_permission": "explicit_user_request"},
        )

        result = adapter.execute_plan(plan, dry_run=True)
        self.assertTrue(result.success)
        self.assertIsNotNone(result.plan)
        assert result.plan is not None
        self.assertEqual(RobotIntentType.SET_JOINT_TARGETS, result.plan.intent_type)

    def test_virtual_openclaw_adapter_dry_run_executes_plan_contract(self) -> None:
        register_builtin_robot_adapters()
        adapter = GLOBAL_ROBOT_ADAPTER_REGISTRY.create("virtual_openclaw")
        plan = RobotActionPlan(
            intent_type=RobotIntentType.SET_JOINT_TARGETS,
            summary="Close index in dry run",
            joint_targets=[RobotJointTarget("index", 20.0)],
            ask_for_confirmation=False,
            metadata={"auto_permission": "explicit_user_request"},
        )

        result = adapter.execute_plan(plan, dry_run=True)
        self.assertTrue(result.success)
        self.assertIsNotNone(result.plan)
        assert result.plan is not None
        self.assertEqual(RobotIntentType.SET_JOINT_TARGETS, result.plan.intent_type)


if __name__ == "__main__":
    unittest.main()
