from __future__ import annotations

from dataclasses import replace

from ..ai_runtime import get_ai_runtime

from ..contracts import (
    AdapterHealthReport,
    RobotActionPlan,
    RobotAdapter,
    RobotCapabilityCatalog,
    RobotExecutionResult,
    RobotIntentType,
    RobotJointTarget,
    RobotState,
    ROBOT_ADAPTER_PROTOCOL_VERSION,
)


_AI = get_ai_runtime()
DEFAULT_GESTURES = _AI.DEFAULT_GESTURES
ExoExecutor = _AI.ExoExecutor
ActionPlan = _AI.ActionPlan
DeviceState = _AI.DeviceState
IntentType = _AI.IntentType
JointTarget = _AI.JointTarget
ExoOrchestrator = _AI.ExoOrchestrator
ExoSafetyValidator = _AI.ExoSafetyValidator


_ROBOT_TO_EXO_INTENT = {
    RobotIntentType.QUERY_STATUS: IntentType.QUERY_STATUS,
    RobotIntentType.SET_JOINT_TARGETS: IntentType.SET_JOINT_ANGLES,
    RobotIntentType.EXECUTE_BEHAVIOR: IntentType.EXECUTE_GESTURE,
    RobotIntentType.CREATE_CUSTOM_BEHAVIOR: IntentType.CREATE_CUSTOM_GESTURE,
    RobotIntentType.RUN_SAVED_BEHAVIOR: IntentType.RUN_SAVED_GESTURE,
    RobotIntentType.HOME: IntentType.HOME,
    RobotIntentType.STOP: IntentType.STOP,
    RobotIntentType.UNKNOWN: IntentType.UNKNOWN,
}


_EXO_TO_ROBOT_INTENT = {value: key for key, value in _ROBOT_TO_EXO_INTENT.items()}


def _to_exo_plan(plan: RobotActionPlan) -> ActionPlan:
    return ActionPlan(
        intent_type=_ROBOT_TO_EXO_INTENT.get(plan.intent_type, IntentType.UNKNOWN),
        summary=plan.summary,
        joint_targets=[
            JointTarget(joint=target.joint, value=target.value, mode=target.mode, note=target.note)
            for target in plan.joint_targets
        ],
        gesture_name=plan.behavior_name,
        gesture_state=plan.behavior_state,
        save_profile_as=plan.save_profile_as,
        ask_for_confirmation=plan.ask_for_confirmation,
        metadata=dict(plan.metadata),
    )


def _from_exo_result(result) -> RobotExecutionResult:
    robot_plan = None
    if result.plan is not None:
        robot_plan = RobotActionPlan(
            intent_type=_EXO_TO_ROBOT_INTENT.get(result.plan.intent_type, RobotIntentType.UNKNOWN),
            summary=result.plan.summary,
            joint_targets=[
                RobotJointTarget(joint=t.joint, value=t.value, mode=t.mode, note=t.note)
                for t in result.plan.joint_targets
            ],
            behavior_name=result.plan.gesture_name,
            behavior_state=result.plan.gesture_state,
            save_profile_as=result.plan.save_profile_as,
            ask_for_confirmation=result.plan.ask_for_confirmation,
            metadata=dict(result.plan.metadata),
        )

    return RobotExecutionResult(
        success=result.success,
        message=result.message,
        command_summary=result.command_summary,
        spoken_response=result.spoken_response,
        plan=robot_plan,
        executed_commands=list(result.executed_commands),
        requires_confirmation=result.requires_confirmation,
        payload=dict(result.payload),
    )


def _from_exo_state(state: DeviceState) -> RobotState:
    return RobotState(
        connected=state.connected,
        robot_mode=state.exo_mode,
        current_behavior=state.current_gesture,
        available_behaviors=list(state.available_gestures),
        joint_limits=dict(state.joint_limits),
        joint_ids=dict(state.joint_ids),
        metadata=dict(state.metadata),
    )


class NmlHandExoRobotAdapter(RobotAdapter):
    def __init__(
        self,
        *,
        exo=None,
        assistant_tone: str = "warm",
        confirmation_mode: str = "balanced",
        profile_root: str | None = None,
    ) -> None:
        self._exo = exo
        self._validator = ExoSafetyValidator(confirmation_mode=confirmation_mode)
        self._executor = ExoExecutor(profile_root=profile_root, assistant_tone=assistant_tone)
        self._state_probe = ExoOrchestrator(exo=exo, validator=self._validator, executor=self._executor)

    @property
    def adapter_id(self) -> str:
        return "nml_hand_exo"

    @property
    def display_name(self) -> str:
        return "NML Hand Exo"

    def describe_capabilities(self) -> RobotCapabilityCatalog:
        state = self.collect_state()
        joint_names = sorted(set(state.joint_limits.keys()) | set(state.joint_ids.keys()))
        if not joint_names:
            joint_names = ["wrist", "thumbflex", "thumbrot", "index", "middle", "ring", "pinky"]

        behaviors = state.available_behaviors or list(DEFAULT_GESTURES)
        return RobotCapabilityCatalog(
            robot_id=self.adapter_id,
            joints=joint_names,
            behaviors=behaviors,
            joint_limits=dict(state.joint_limits),
            supported_intents=list(RobotIntentType),
            protocol_version=ROBOT_ADAPTER_PROTOCOL_VERSION,
            metadata={"display_name": self.display_name},
        )

    def initialize(self) -> None:
        if self._exo and hasattr(self._exo, "connect") and not getattr(self._exo, "connected", False):
            self._exo.connect()

    def health_check(self) -> AdapterHealthReport:
        connected = bool(getattr(self._exo, "connected", False)) if self._exo is not None else False
        if self._exo is None:
            # Allow dry-run planning setups where hardware is intentionally absent.
            return AdapterHealthReport(healthy=True, message="No runtime device attached; dry-run mode assumed.")
        if not connected:
            return AdapterHealthReport(healthy=False, message="NML device is not connected.")
        return AdapterHealthReport(healthy=True, details={"connected": str(connected).lower()})

    def collect_state(self) -> RobotState:
        exo_state = self._state_probe.collect_device_state()
        return _from_exo_state(exo_state)

    def execute_plan(self, plan: RobotActionPlan, *, dry_run: bool = True) -> RobotExecutionResult:
        exo_plan = _to_exo_plan(plan)
        exo_state = self._state_probe.collect_device_state()
        validated_plan = self._validator.validate(exo_plan, exo_state)
        exo_result = self._executor.execute(validated_plan, exo=self._exo, device_state=exo_state, dry_run=dry_run)

        # Preserve any upstream metadata while keeping validated values from the exo path.
        if exo_result.plan is not None:
            exo_result.plan = replace(exo_result.plan, metadata={**plan.metadata, **exo_result.plan.metadata})

        return _from_exo_result(exo_result)

    def close(self) -> None:
        if self._exo and hasattr(self._exo, "close"):
            self._exo.close()

    def shutdown(self) -> None:
        self.close()
