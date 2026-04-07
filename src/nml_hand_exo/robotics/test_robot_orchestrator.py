from __future__ import annotations

import unittest

from nml_hand_exo.robotics.bootstrap import register_builtin_robot_adapters
from nml_hand_exo.robotics.registry import GLOBAL_ROBOT_ADAPTER_REGISTRY
from nml_hand_exo.robotics.robot_orchestrator import RobotOrchestrator


class RobotOrchestratorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from nml_hand_exo.robotics.ai_runtime import get_ai_runtime

        try:
            ai_runtime = get_ai_runtime()
        except ModuleNotFoundError as exc:
            raise unittest.SkipTest(f"NeuroBridge runtime unavailable for robotics tests: {exc}")

        cls.HeuristicIntentProvider = ai_runtime.HeuristicIntentProvider
        cls.TelemetryLogger = ai_runtime.TelemetryLogger
        register_builtin_robot_adapters()

    def test_robot_orchestrator_executes_status_via_adapter(self) -> None:
        adapter = GLOBAL_ROBOT_ADAPTER_REGISTRY.create("virtual_openclaw")
        orchestrator = RobotOrchestrator(
            adapter=adapter,
            provider=self.HeuristicIntentProvider(),
            telemetry=self.TelemetryLogger(),
        )

        result = orchestrator.handle_input("status", dry_run=False)
        self.assertTrue(result.success)
        self.assertIn("Connected", result.voice_text())

        orchestrator.close()


if __name__ == "__main__":
    unittest.main()
