from __future__ import annotations

from dataclasses import dataclass, field

from .contracts import RobotActionPlan, RobotCapabilityCatalog, RobotState


@dataclass
class PolicyDecision:
    allow: bool
    message: str = ""
    requires_confirmation: bool = False
    metadata: dict[str, str] = field(default_factory=dict)


class SafetyPolicyRule:
    def evaluate(
        self,
        plan: RobotActionPlan,
        *,
        capabilities: RobotCapabilityCatalog,
        state: RobotState,
    ) -> PolicyDecision:
        return PolicyDecision(allow=True)


class JointSupportRule(SafetyPolicyRule):
    def evaluate(
        self,
        plan: RobotActionPlan,
        *,
        capabilities: RobotCapabilityCatalog,
        state: RobotState,
    ) -> PolicyDecision:
        supported = set(capabilities.joints)
        for target in plan.joint_targets:
            if target.joint not in supported:
                return PolicyDecision(
                    allow=False,
                    message=f"Unsupported joint target for adapter '{capabilities.robot_id}': {target.joint}",
                )
        return PolicyDecision(allow=True)


class JointLimitRule(SafetyPolicyRule):
    def __init__(self, max_relative_angle: float = 180.0) -> None:
        self.max_relative_angle = max_relative_angle

    def evaluate(
        self,
        plan: RobotActionPlan,
        *,
        capabilities: RobotCapabilityCatalog,
        state: RobotState,
    ) -> PolicyDecision:
        limits = capabilities.joint_limits or state.joint_limits
        for target in plan.joint_targets:
            if target.mode == "absolute" and target.joint in limits:
                lower, upper = limits[target.joint]
                if not lower <= target.value <= upper:
                    return PolicyDecision(
                        allow=False,
                        message=(
                            f"Absolute target {target.value} for {target.joint} is outside limits "
                            f"[{lower}, {upper}]"
                        ),
                    )
            elif target.mode == "relative" and abs(target.value) > self.max_relative_angle:
                return PolicyDecision(
                    allow=False,
                    message=(
                        f"Relative target {target.value} for {target.joint} exceeds "
                        f"software limit {self.max_relative_angle}"
                    ),
                )
        return PolicyDecision(allow=True)


class ConfirmationRule(SafetyPolicyRule):
    def __init__(self, mode: str = "balanced") -> None:
        normalized = str(mode or "balanced").strip().lower()
        self.mode = normalized if normalized in {"strict", "balanced", "relaxed"} else "balanced"

    def evaluate(
        self,
        plan: RobotActionPlan,
        *,
        capabilities: RobotCapabilityCatalog,
        state: RobotState,
    ) -> PolicyDecision:
        if self.mode == "strict":
            return PolicyDecision(allow=True, requires_confirmation=True)

        if self.mode == "balanced":
            if len(plan.joint_targets) > 1 and not plan.metadata.get("auto_permission"):
                return PolicyDecision(allow=True, requires_confirmation=True)
            return PolicyDecision(allow=True, requires_confirmation=plan.ask_for_confirmation)

        # relaxed
        if plan.metadata.get("clarification_kind"):
            return PolicyDecision(allow=True, requires_confirmation=True)
        return PolicyDecision(allow=True, requires_confirmation=plan.ask_for_confirmation)


class RobotSafetyPolicyEngine:
    def __init__(self, rules: list[SafetyPolicyRule] | None = None) -> None:
        self.rules = rules or [JointSupportRule(), JointLimitRule(), ConfirmationRule()]

    def evaluate(
        self,
        plan: RobotActionPlan,
        *,
        capabilities: RobotCapabilityCatalog,
        state: RobotState,
    ) -> PolicyDecision:
        requires_confirmation = bool(plan.ask_for_confirmation)
        merged_metadata: dict[str, str] = {}
        for rule in self.rules:
            decision = rule.evaluate(plan, capabilities=capabilities, state=state)
            if decision.metadata:
                merged_metadata.update(decision.metadata)
            if not decision.allow:
                return decision
            requires_confirmation = requires_confirmation or decision.requires_confirmation

        return PolicyDecision(
            allow=True,
            requires_confirmation=requires_confirmation,
            metadata=merged_metadata,
        )
