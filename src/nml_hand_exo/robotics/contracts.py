from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


ROBOT_ADAPTER_PROTOCOL_VERSION = 1


class RobotIntentType(str, Enum):
    QUERY_STATUS = "query_status"
    SET_JOINT_TARGETS = "set_joint_targets"
    EXECUTE_BEHAVIOR = "execute_behavior"
    CREATE_CUSTOM_BEHAVIOR = "create_custom_behavior"
    RUN_SAVED_BEHAVIOR = "run_saved_behavior"
    HOME = "home"
    STOP = "stop"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class RobotJointTarget:
    joint: str
    value: float
    mode: str = "relative"
    note: str = ""


@dataclass
class RobotCapabilityCatalog:
    robot_id: str
    joints: list[str] = field(default_factory=list)
    behaviors: list[str] = field(default_factory=list)
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    supported_intents: list[RobotIntentType] = field(default_factory=list)
    protocol_version: int = ROBOT_ADAPTER_PROTOCOL_VERSION
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class AdapterCompatibilityReport:
    compatible: bool
    message: str = ""
    protocol_version: int = ROBOT_ADAPTER_PROTOCOL_VERSION
    supported_intents: tuple[RobotIntentType, ...] = tuple()


@dataclass(frozen=True)
class AdapterHealthReport:
    healthy: bool
    message: str = ""
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class RobotState:
    connected: bool = False
    robot_mode: str = ""
    current_behavior: str = ""
    available_behaviors: list[str] = field(default_factory=list)
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    joint_ids: dict[str, int] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class RobotActionPlan:
    intent_type: RobotIntentType
    summary: str
    joint_targets: list[RobotJointTarget] = field(default_factory=list)
    behavior_name: str | None = None
    behavior_state: str | None = None
    save_profile_as: str | None = None
    ask_for_confirmation: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["intent_type"] = self.intent_type.value
        return payload


@dataclass
class RobotExecutionResult:
    success: bool
    message: str
    command_summary: str = ""
    spoken_response: str = ""
    plan: RobotActionPlan | None = None
    executed_commands: list[str] = field(default_factory=list)
    requires_confirmation: bool = False
    payload: dict[str, Any] = field(default_factory=dict)


class RobotAdapter(ABC):
    """Robot-hardware abstraction for portable deployment across devices."""

    @property
    @abstractmethod
    def adapter_id(self) -> str:
        raise NotImplementedError()

    @property
    @abstractmethod
    def display_name(self) -> str:
        raise NotImplementedError()

    @abstractmethod
    def describe_capabilities(self) -> RobotCapabilityCatalog:
        raise NotImplementedError()

    @abstractmethod
    def collect_state(self) -> RobotState:
        raise NotImplementedError()

    @abstractmethod
    def execute_plan(self, plan: RobotActionPlan, *, dry_run: bool = True) -> RobotExecutionResult:
        raise NotImplementedError()

    def initialize(self) -> None:
        return

    def health_check(self) -> AdapterHealthReport:
        return AdapterHealthReport(healthy=True)

    def compatibility_report(self) -> AdapterCompatibilityReport:
        capabilities = self.describe_capabilities()
        supported_intents = tuple(capabilities.supported_intents)
        protocol_version = int(capabilities.protocol_version)

        if protocol_version != ROBOT_ADAPTER_PROTOCOL_VERSION:
            return AdapterCompatibilityReport(
                compatible=False,
                message=(
                    f"Adapter protocol mismatch. Expected {ROBOT_ADAPTER_PROTOCOL_VERSION}, "
                    f"got {protocol_version}."
                ),
                protocol_version=protocol_version,
                supported_intents=supported_intents,
            )

        required_intents = {
            RobotIntentType.QUERY_STATUS,
            RobotIntentType.SET_JOINT_TARGETS,
            RobotIntentType.HOME,
            RobotIntentType.STOP,
        }
        if not required_intents.issubset(set(supported_intents)):
            missing = sorted(intent.value for intent in required_intents.difference(set(supported_intents)))
            return AdapterCompatibilityReport(
                compatible=False,
                message=f"Adapter missing required intents: {', '.join(missing)}",
                protocol_version=protocol_version,
                supported_intents=supported_intents,
            )

        return AdapterCompatibilityReport(
            compatible=True,
            protocol_version=protocol_version,
            supported_intents=supported_intents,
        )

    def close(self) -> None:
        return

    def shutdown(self) -> None:
        self.close()
