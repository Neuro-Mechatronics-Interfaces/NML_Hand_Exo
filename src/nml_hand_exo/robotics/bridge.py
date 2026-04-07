from __future__ import annotations

"""Transitional bridge between exo robotics contracts and NeuroBridge AI models.

Phase 1: kept in NML_Hand_Exo to avoid runtime breakage.
Phase 2 target: move canonical mapping implementation into NeuroBridge,
leaving only a thin compatibility shim here.
"""

from .ai_runtime import get_ai_runtime, warn_transitional_owner

from .contracts import RobotActionPlan, RobotExecutionResult, RobotIntentType, RobotJointTarget, RobotState


_AI = get_ai_runtime()
warn_transitional_owner(__name__)
ActionPlan = _AI.ActionPlan
DeviceState = _AI.DeviceState
IntentType = _AI.IntentType
JointTarget = _AI.JointTarget
OrchestratorResult = _AI.OrchestratorResult


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


def ai_plan_to_robot_plan(plan: ActionPlan) -> RobotActionPlan:
    return RobotActionPlan(
        intent_type=_EXO_TO_ROBOT_INTENT.get(plan.intent_type, RobotIntentType.UNKNOWN),
        summary=plan.summary,
        joint_targets=[
            RobotJointTarget(joint=target.joint, value=target.value, mode=target.mode, note=target.note)
            for target in plan.joint_targets
        ],
        behavior_name=plan.gesture_name,
        behavior_state=plan.gesture_state,
        save_profile_as=plan.save_profile_as,
        ask_for_confirmation=plan.ask_for_confirmation,
        metadata=dict(plan.metadata),
    )


def robot_plan_to_ai_plan(plan: RobotActionPlan) -> ActionPlan:
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


def robot_state_to_ai_state(state: RobotState) -> DeviceState:
    return DeviceState(
        connected=state.connected,
        exo_mode=state.robot_mode,
        current_gesture=state.current_behavior,
        available_gestures=list(state.available_behaviors),
        joint_limits=dict(state.joint_limits),
        joint_ids=dict(state.joint_ids),
        metadata=dict(state.metadata),
    )


def robot_result_to_ai_result(result: RobotExecutionResult) -> OrchestratorResult:
    plan = robot_plan_to_ai_plan(result.plan) if result.plan is not None else None
    return OrchestratorResult(
        success=result.success,
        message=result.message,
        command_summary=result.command_summary,
        spoken_response=result.spoken_response,
        plan=plan,
        executed_commands=list(result.executed_commands),
        requires_confirmation=result.requires_confirmation,
        payload=dict(result.payload),
    )
