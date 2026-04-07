# NeuroBridge Boundary for NML HandExo

This document defines what remains in NML_Hand_Exo and what should evolve inside NeuroBridge when using NeuroBridge as an AI dependency/submodule.

Use this with `docs/PHASE1_MIGRATION_CHECKLIST.md` for initial migration and `docs/PHASE2_CUTOVER_CHECKLIST.md` for ownership consolidation.

## Keep in NML_Hand_Exo

- Hardware/transport interfaces and exo control APIs
  - src/nml_hand_exo/interface
  - src/nml_hand_exo/control
- Firmware and low-level C++ code
  - src/cpp
- Calibration data, hardware profiles, and motor limits
  - calibration
  - config/robot_adapters
- Device-specific examples and protocol-facing docs
  - examples focused on hardware control
  - core exo API docs

## Move or Treat as NeuroBridge-Owned

- AI conversation orchestration lifecycle
- AI app runner/entrypoint UX and shell scripts
- AI provider strategy and runtime composition defaults
- Deployment profile strategy for AI scenarios

When submodule mode is active, these are expected to come from:
- external/NeuroBridge

## Shared/Bridged Layer

- Robotics adapter contract integration for exo device control
- Bundle profile wiring for runtime defaults
- Telemetry/visualization protocol compatibility

## Phase 1 Migration Status (No Breakage)

### Move Now (Low Risk)

- Prompt-engineering assets and generic LLM conversation tuning:
  - .github/prompts/agent-prompt-optimizer.prompt.md
  - .github/prompts/conversation-repair-prompting.prompt.md
  - .github/prompts/naturalistic-prompt-suite.prompt.md

### Keep in NML_Hand_Exo (Authoritative)

- Hardware/safety and adapter boundary:
  - src/nml_hand_exo/robotics/contracts.py
  - src/nml_hand_exo/robotics/policy.py
  - src/nml_hand_exo/robotics/config.py
  - src/nml_hand_exo/robotics/registry.py
  - src/nml_hand_exo/robotics/bootstrap.py
  - src/nml_hand_exo/robotics/adapters/nml_hand_exo_adapter.py
- Device-bound runtime configuration and safety docs:
  - config/robot_adapters/nml_hand_exo.yaml
  - config/bundles/nml_default.yaml
  - config/policy_profiles/safe_lab.yaml
  - docs/SAFETY_REQUIREMENTS.md

### Keep Temporarily (Transitional)

- Thin wrappers and bridge modules that currently preserve compatibility:
  - src/nml_hand_exo/robotics/bridge.py
  - src/nml_hand_exo/robotics/ai_runtime.py
  - src/nml_hand_exo/robotics/robot_orchestrator.py
  - src/nml_hand_exo/robotics/adapters/virtual_openclaw_adapter.py
  - scripts/setup_ai_submodule_env.py
  - scripts/run_ai_agent.py
  - scripts/run_exo_visualizer.py

These transitional modules can be reduced to shims in Phase 2 once NeuroBridge hosts the canonical runtime implementation.

## Phase 2 Status (In Progress)

- Transitional runtime modules remain available for compatibility.
- Phase-2 warning surface should be enabled before removing duplicate runtime logic.
- Final duplicate-removal cut should happen only after one compatibility cycle.

## Script Cleanup Guidance

- Keep BAT + Python launch/setup scripts (cross-terminal and policy-safe).
- PowerShell helper scripts were removed to enforce one standard launch path.
- Keep helper launchers that reduce onboarding friction:
  - scripts/setup_neurobridge_submodule.bat
  - scripts/setup_ai_submodule_env.py
  - scripts/run_ai_agent.bat
  - scripts/run_exo_visualizer.bat

## Do We Still Need "Test Scripts"?

- Runtime launch helper scripts: yes, keep (they are onboarding scripts, not test scripts).
- One-off experimental scripts: remove or archive after validated behavior is merged.
- Automated tests: keep in source test suites; do not rely on ad-hoc manual scripts for regression safety.
