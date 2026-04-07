from __future__ import annotations

from .contracts import (
    AdapterHealthReport,
    RobotActionPlan,
    RobotAdapter,
    RobotCapabilityCatalog,
    RobotExecutionResult,
    RobotIntentType,
    RobotJointTarget,
    RobotState,
)
from .config import RobotAdapterRuntimeConfig, apply_cli_defaults_from_config, load_robot_adapter_config
from .bundles import (
    DEPLOYMENT_BUNDLE_VERSION,
    DeploymentBundleManifest,
    apply_bundle_defaults,
    list_available_bundles,
    load_bundle_manifest,
)
from .pipeline import PipelineEvent, PipelineStage, PipelineTrace
from .policy import (
    ConfirmationRule,
    JointLimitRule,
    JointSupportRule,
    PolicyDecision,
    RobotSafetyPolicyEngine,
)
from .policy_profiles import (
    POLICY_PROFILE_VERSION,
    PolicyProfileManifest,
    apply_policy_profile_defaults,
    list_available_policy_profiles,
    load_policy_profile_manifest,
)
from .registry import GLOBAL_ROBOT_ADAPTER_REGISTRY, RobotAdapterManifest, RobotAdapterRegistry

# AI-runtime dependent robotics helpers are optional at import time.
# This allows config/manifest test suites to run in environments where
# NeuroBridge AI modules are intentionally unavailable.
_AI_RUNTIME_IMPORT_ERROR = None
try:
    from .bridge import ai_plan_to_robot_plan, robot_plan_to_ai_plan, robot_result_to_ai_result, robot_state_to_ai_state
    from .robot_orchestrator import RobotOrchestrator
except Exception as exc:
    _AI_RUNTIME_IMPORT_ERROR = exc

__all__ = [
    "ConfirmationRule",
    "GLOBAL_ROBOT_ADAPTER_REGISTRY",
    "AdapterHealthReport",
    "DEPLOYMENT_BUNDLE_VERSION",
    "DeploymentBundleManifest",
    "JointLimitRule",
    "JointSupportRule",
    "PipelineEvent",
    "PipelineStage",
    "PipelineTrace",
    "POLICY_PROFILE_VERSION",
    "PolicyDecision",
    "PolicyProfileManifest",
    "RobotAdapterRuntimeConfig",
    "RobotActionPlan",
    "RobotAdapter",
    "RobotAdapterManifest",
    "RobotAdapterRegistry",
    "RobotCapabilityCatalog",
    "RobotExecutionResult",
    "RobotIntentType",
    "RobotJointTarget",
    "RobotSafetyPolicyEngine",
    "RobotState",
    "apply_cli_defaults_from_config",
    "apply_bundle_defaults",
    "apply_policy_profile_defaults",
    "list_available_bundles",
    "list_available_policy_profiles",
    "load_robot_adapter_config",
    "load_bundle_manifest",
    "load_policy_profile_manifest",
]

if _AI_RUNTIME_IMPORT_ERROR is None:
    __all__ += [
        "RobotOrchestrator",
        "ai_plan_to_robot_plan",
        "robot_plan_to_ai_plan",
        "robot_result_to_ai_result",
        "robot_state_to_ai_state",
    ]
