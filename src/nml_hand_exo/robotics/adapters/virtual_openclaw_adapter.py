from __future__ import annotations

"""Transitional cross-robot virtual adapter implementation.

Phase 1: retained in NML_Hand_Exo so tests and demos remain stable.
Phase 2 target: move canonical virtual adapter patterns to NeuroBridge
or a shared robotics runtime package.
"""

from dataclasses import dataclass, field

from ..ai_runtime import get_ai_runtime, warn_transitional_owner

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
warn_transitional_owner(__name__)
DEFAULT_GESTURES = _AI.DEFAULT_GESTURES
gesture_level_to_ratio = _AI.gesture_level_to_ratio
interpolate_gesture_targets = _AI.interpolate_gesture_targets
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


@dataclass
class VirtualOpenClawDevice:
    """OpenClaw-style virtual robot with an exo-compatible control surface."""

    name: str = "OpenClaw Virtual Robot"
    connected: bool = False
    exo_mode: str = "virtual_openclaw"
    current_gesture: str = ""
    current_gesture_state: str = "open"
    joint_positions: dict[str, float] = field(
        default_factory=lambda: {
            "wrist": 0.0,
            "thumbflex": 0.0,
            "thumbrot": 140.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "pinky": 0.0,
        }
    )
    command_log: list[str] = field(default_factory=list)

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def info(self) -> dict:
        motors = {
            str(index): {"name": name}
            for index, name in enumerate(self.joint_positions.keys(), start=1)
        }
        return {
            "name": self.name,
            "n_motors": len(motors),
            "motors": motors,
            "robot_family": "openclaw",
        }

    def get_exo_mode(self) -> str:
        return self.exo_mode

    def get_gesture(self) -> str:
        if not self.current_gesture:
            return "none"
        return f"{self.current_gesture}:{self.current_gesture_state}"

    def get_gesture_list(self) -> list[str]:
        return DEFAULT_GESTURES.copy()

    def get_motor_limits(self, motor) -> dict[str, tuple[float, float]]:
        limits = {
            str(index): (-90.0, 180.0)
            for index, _name in enumerate(self.joint_positions.keys(), start=1)
        }
        if motor == "all":
            return limits
        return {str(motor): limits.get(str(motor), (-90.0, 180.0))}

    def home(self, _motor_group: str) -> None:
        for joint in self.joint_positions:
            self.joint_positions[joint] = 0.0 if joint != "thumbrot" else 140.0
        self.current_gesture = ""
        self.current_gesture_state = "open"
        self.command_log.append("home:all")

    def set_gesture(self, gesture_name: str, gesture_state: str = "default") -> None:
        ratio = gesture_level_to_ratio(gesture_state)
        if ratio is None:
            ratio = gesture_level_to_ratio("default") or 0.5

        for target in interpolate_gesture_targets(gesture_name, ratio):
            self.joint_positions[target.joint] = target.value

        self.current_gesture = gesture_name
        self.current_gesture_state = gesture_state
        self.command_log.append(f"set_gesture:{gesture_name}:{gesture_state}")

    def set_motor_angle(self, joint_or_index, value: float) -> None:
        joint_name = self._resolve_joint_name(joint_or_index)
        self.joint_positions[joint_name] = float(value)
        self.command_log.append(f"set_angle:{joint_name}:{float(value)}")

    def set_absolute_motor_angle(self, joint_or_index, value: float) -> None:
        joint_name = self._resolve_joint_name(joint_or_index)
        self.joint_positions[joint_name] = float(value)
        self.command_log.append(f"set_absolute_angle:{joint_name}:{float(value)}")

    def snapshot(self) -> dict:
        return {
            "connected": self.connected,
            "exo_mode": self.exo_mode,
            "current_gesture": self.current_gesture,
            "current_gesture_state": self.current_gesture_state,
            "joint_positions": self.joint_positions.copy(),
            "robot_family": "openclaw",
            "command_log": self.command_log.copy(),
        }

    def _resolve_joint_name(self, joint_or_index) -> str:
        if isinstance(joint_or_index, int) or str(joint_or_index).isdigit():
            index = int(joint_or_index) - 1
            joint_names = list(self.joint_positions.keys())
            if 0 <= index < len(joint_names):
                return joint_names[index]
            raise KeyError(f"Unknown virtual motor index: {joint_or_index}")

        joint_name = str(joint_or_index).strip().lower()
        if joint_name not in self.joint_positions:
            raise KeyError(f"Unknown virtual joint: {joint_or_index}")
        return joint_name


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


class VirtualOpenClawRobotAdapter(RobotAdapter):
    def __init__(
        self,
        *,
        device: VirtualOpenClawDevice | None = None,
        assistant_tone: str = "warm",
        confirmation_mode: str = "balanced",
        profile_root: str | None = None,
    ) -> None:
        if isinstance(device, dict):
            self.device = VirtualOpenClawDevice(**device)
        else:
            self.device = device or VirtualOpenClawDevice()
        if not self.device.connected:
            self.device.connect()

        self._validator = ExoSafetyValidator(confirmation_mode=confirmation_mode)
        self._executor = ExoExecutor(profile_root=profile_root, assistant_tone=assistant_tone)
        self._state_probe = ExoOrchestrator(exo=self.device, validator=self._validator, executor=self._executor)

    @property
    def adapter_id(self) -> str:
        return "virtual_openclaw"

    @property
    def display_name(self) -> str:
        return "Virtual OpenClaw"

    def describe_capabilities(self) -> RobotCapabilityCatalog:
        state = self.collect_state()
        joint_names = sorted(set(state.joint_limits.keys()) | set(state.joint_ids.keys()))
        if not joint_names:
            joint_names = ["wrist", "thumbflex", "thumbrot", "index", "middle", "ring", "pinky"]

        return RobotCapabilityCatalog(
            robot_id=self.adapter_id,
            joints=joint_names,
            behaviors=state.available_behaviors or list(DEFAULT_GESTURES),
            joint_limits=dict(state.joint_limits),
            supported_intents=list(RobotIntentType),
            protocol_version=ROBOT_ADAPTER_PROTOCOL_VERSION,
            metadata={"display_name": self.display_name, "robot_family": "openclaw"},
        )

    def initialize(self) -> None:
        if not self.device.connected:
            self.device.connect()

    def health_check(self) -> AdapterHealthReport:
        if not self.device.connected:
            return AdapterHealthReport(healthy=False, message="Virtual OpenClaw device is disconnected.")
        return AdapterHealthReport(healthy=True, details={"connected": "true"})

    def collect_state(self) -> RobotState:
        exo_state = self._state_probe.collect_device_state()
        return _from_exo_state(exo_state)

    def execute_plan(self, plan: RobotActionPlan, *, dry_run: bool = True) -> RobotExecutionResult:
        exo_plan = _to_exo_plan(plan)
        exo_state = self._state_probe.collect_device_state()
        validated_plan = self._validator.validate(exo_plan, exo_state)
        exo_result = self._executor.execute(validated_plan, exo=self.device, device_state=exo_state, dry_run=dry_run)
        return _from_exo_result(exo_result)

    def close(self) -> None:
        self.device.close()

    def shutdown(self) -> None:
        self.close()
