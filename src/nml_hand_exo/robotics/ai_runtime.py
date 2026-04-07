from __future__ import annotations

"""Transitional NeuroBridge runtime loader for robotics modules.

Phase 1: retained to preserve local integration behavior.
Phase 2 target: move runtime resolution into NeuroBridge and keep
this module as a minimal compatibility layer.
"""

from importlib import import_module
from pathlib import Path
import sys
from types import SimpleNamespace
import warnings


_WARNED_TRANSITIONAL_MODULES: set[str] = set()


def _bootstrap_neurobridge_src() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    candidates = [
        repo_root / "external" / "NeuroBridge" / "src",
        repo_root.parent / "NeuroBridge" / "src",
    ]
    for src_path in candidates:
        if not src_path.exists():
            continue
        src_path_str = str(src_path)
        if src_path_str in sys.path:
            sys.path.remove(src_path_str)
        sys.path.insert(0, src_path_str)
        return


_bootstrap_neurobridge_src()


def _import_attr(module_path: str, attribute: str):
    try:
        module = import_module(module_path)
    except ModuleNotFoundError as exc:
        _bootstrap_neurobridge_src()
        try:
            module = import_module(module_path)
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                "AI runtime is not available. Initialize NeuroBridge and install the AI dependency "
                "(see docs/AI_SUBMODULE_SETUP.md)."
            ) from exc

    return getattr(module, attribute)


def warn_transitional_owner(module_name: str) -> None:
    key = str(module_name).strip().lower()
    if not key or key in _WARNED_TRANSITIONAL_MODULES:
        return

    _WARNED_TRANSITIONAL_MODULES.add(key)
    warnings.warn(
        (
            f"{module_name} is transitional in NML_Hand_Exo and is targeted to become a thin shim. "
            "NeuroBridge is the canonical owner for AI runtime/orchestration behavior."
        ),
        DeprecationWarning,
        stacklevel=2,
    )


def get_ai_runtime() -> SimpleNamespace:
    return SimpleNamespace(
        DEFAULT_GESTURES=_import_attr("nml_hand_exo.ai.capabilities", "DEFAULT_GESTURES"),
        gesture_level_to_ratio=_import_attr("nml_hand_exo.ai.capabilities", "gesture_level_to_ratio"),
        interpolate_gesture_targets=_import_attr("nml_hand_exo.ai.capabilities", "interpolate_gesture_targets"),
        ExoExecutor=_import_attr("nml_hand_exo.ai.executor", "ExoExecutor"),
        ActionPlan=_import_attr("nml_hand_exo.ai.models", "ActionPlan"),
        DeviceState=_import_attr("nml_hand_exo.ai.models", "DeviceState"),
        IntentType=_import_attr("nml_hand_exo.ai.models", "IntentType"),
        JointTarget=_import_attr("nml_hand_exo.ai.models", "JointTarget"),
        OrchestratorResult=_import_attr("nml_hand_exo.ai.models", "OrchestratorResult"),
        ExoOrchestrator=_import_attr("nml_hand_exo.ai.orchestrator", "ExoOrchestrator"),
        IntentProvider=_import_attr("nml_hand_exo.ai.providers.base", "IntentProvider"),
        HeuristicIntentProvider=_import_attr("nml_hand_exo.ai.providers.heuristic_provider", "HeuristicIntentProvider"),
        ExoSafetyValidator=_import_attr("nml_hand_exo.ai.safety", "ExoSafetyValidator"),
        TelemetryLogger=_import_attr("nml_hand_exo.ai.telemetry", "TelemetryLogger"),
        SimulatedHandExo=_import_attr("nml_hand_exo.ai.simulator", "SimulatedHandExo"),
    )
