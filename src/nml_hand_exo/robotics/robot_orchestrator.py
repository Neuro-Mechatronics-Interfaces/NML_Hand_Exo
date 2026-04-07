from __future__ import annotations

"""Transitional robot-neutral orchestrator facade.

Phase 1: remains in NML_Hand_Exo for interface stability.
Phase 2 target: move canonical orchestration logic into NeuroBridge
and keep only adapter-facing glue in this repository.
"""

from collections.abc import Iterator

from .ai_runtime import get_ai_runtime, warn_transitional_owner

from .bridge import ai_plan_to_robot_plan, robot_result_to_ai_result, robot_state_to_ai_state
from .contracts import RobotAdapter


_AI = get_ai_runtime()
warn_transitional_owner(__name__)
IntentType = _AI.IntentType
OrchestratorResult = _AI.OrchestratorResult
ExoOrchestrator = _AI.ExoOrchestrator
IntentProvider = _AI.IntentProvider
HeuristicIntentProvider = _AI.HeuristicIntentProvider
ExoSafetyValidator = _AI.ExoSafetyValidator
TelemetryLogger = _AI.TelemetryLogger


class RobotOrchestrator:
    """Robot-neutral orchestration boundary that plans in AI space and executes via RobotAdapter."""

    def __init__(
        self,
        *,
        adapter: RobotAdapter,
        provider: IntentProvider | None = None,
        validator: ExoSafetyValidator | None = None,
        telemetry: TelemetryLogger | None = None,
        assistant_tone: str = "warm",
    ) -> None:
        self.adapter = adapter
        self.telemetry = telemetry or TelemetryLogger()
        self.exo = self._resolve_runtime_surface(adapter)
        self._planner = ExoOrchestrator(
            exo=None,
            provider=provider or HeuristicIntentProvider(),
            validator=validator or ExoSafetyValidator(),
            telemetry=self.telemetry,
            assistant_tone=assistant_tone,
        )

    @staticmethod
    def _resolve_runtime_surface(adapter: RobotAdapter):
        if hasattr(adapter, "device"):
            return getattr(adapter, "device")
        if hasattr(adapter, "_exo"):
            return getattr(adapter, "_exo")
        return None

    def collect_device_state(self):
        return robot_state_to_ai_state(self.adapter.collect_state())

    def plan(self, user_text: str):
        return self._planner.plan(user_text)

    def handle_input(self, user_text: str, dry_run: bool = True, response_user_text: str | None = None) -> OrchestratorResult:
        planned = self._planner.handle_input(user_text, dry_run=True, response_user_text=response_user_text)
        if planned.plan is None:
            return planned

        plan = planned.plan
        if plan.intent_type == IntentType.UNKNOWN:
            return planned

        if planned.requires_confirmation and not dry_run:
            return planned

        if dry_run and plan.intent_type != IntentType.QUERY_STATUS:
            return planned

        robot_plan = ai_plan_to_robot_plan(plan)
        adapter_result = self.adapter.execute_plan(robot_plan, dry_run=dry_run and plan.intent_type != IntentType.QUERY_STATUS)
        ai_result = robot_result_to_ai_result(adapter_result)

        if ai_result.plan is not None:
            ai_result.plan.metadata = {**dict(plan.metadata), **dict(ai_result.plan.metadata)}

        ai_result.payload["provider_used"] = str(plan.metadata.get("provider_used", "adapter"))
        self.telemetry.log_event(
            "robot_execution_dispatched",
            adapter_id=self.adapter.adapter_id,
            intent_type=plan.intent_type.value,
            dry_run=dry_run,
            success=ai_result.success,
            command_summary=ai_result.command_text(),
        )
        return ai_result

    def execute_plan(self, plan, dry_run: bool = False) -> OrchestratorResult:
        robot_plan = ai_plan_to_robot_plan(plan)
        adapter_result = self.adapter.execute_plan(robot_plan, dry_run=dry_run)
        return robot_result_to_ai_result(adapter_result)

    def broadcast_device_state(
        self,
        *,
        provider_used: str = "",
        event_type: str = "device_state",
        status_text: str = "",
        command_summary: str = "",
    ) -> dict:
        state = self.adapter.collect_state()
        event = self.telemetry.broadcast_live_event(
            event_type,
            provider_used=provider_used,
            command_summary=command_summary,
            spoken_response=status_text,
            device_snapshot={
                "connected": state.connected,
                "exo_mode": state.robot_mode,
                "current_gesture": state.current_behavior,
                "joint_positions": state.metadata.get("joint_positions", {}),
                "adapter_id": self.adapter.adapter_id,
            },
        )
        return event

    def stream_spoken_reply(
        self,
        user_text: str,
        result,
        *,
        dry_run: bool,
        executed: bool,
    ) -> Iterator[str]:
        if not result.plan or not result.success:
            yield result.voice_text()
            return

        state = self.collect_device_state()
        yield from self._planner.provider.stream_assistant_reply(
            user_text,
            result.plan,
            state,
            dry_run=dry_run,
            requires_confirmation=result.requires_confirmation,
            executed=executed,
        )

    def close(self) -> None:
        self.adapter.shutdown()
