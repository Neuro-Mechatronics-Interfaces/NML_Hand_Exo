# Phase 2 Cutover Checklist (Ownership Consolidation)

This checklist reduces transitional duplication by making NeuroBridge the canonical home for AI runtime and orchestration behavior.

## Goal

- Keep NML_Hand_Exo authoritative for hardware, safety, adapter contracts, and manifests.
- Treat NeuroBridge as canonical for AI runtime/orchestration implementation.
- Preserve compatibility for existing imports during migration.

## Preconditions

- `external/NeuroBridge` is present and initialized.
- Phase 1 checklist items are complete.
- Validation tests in `src/nml_hand_exo/robotics/test_*.py` pass.

## Step 1: Enforce Transitional Warning Surface

Keep compatibility modules, but emit explicit migration warnings from:

- `src/nml_hand_exo/robotics/bridge.py`
- `src/nml_hand_exo/robotics/robot_orchestrator.py`
- `src/nml_hand_exo/robotics/adapters/virtual_openclaw_adapter.py`

These warnings should point consumers to NeuroBridge as the canonical runtime owner.

## Step 2: Keep Exo Authority Boundaries Fixed

Do not move these files in Phase 2:

- `src/nml_hand_exo/robotics/contracts.py`
- `src/nml_hand_exo/robotics/policy.py`
- `src/nml_hand_exo/robotics/config.py`
- `src/nml_hand_exo/robotics/registry.py`
- `src/nml_hand_exo/robotics/bootstrap.py`
- `src/nml_hand_exo/robotics/adapters/nml_hand_exo_adapter.py`
- `config/robot_adapters/nml_hand_exo.yaml`
- `config/bundles/nml_default.yaml`
- `config/policy_profiles/safe_lab.yaml`

## Step 3: Convert Transitional Modules to Thin Shims

When NeuroBridge runtime parity is confirmed, reduce transitional modules to import-forwarding shims and keep only adapter-facing glue in NML_Hand_Exo.

## Step 4: Remove Duplicate Runtime Logic (Final Cut)

After one full release cycle of shim warnings, remove duplicate AI/runtime implementation from NML_Hand_Exo while preserving public import compatibility where feasible.

## Validation Gate

Run all of the following before each cutover commit:

```bash
python src/nml_hand_exo/robotics/test_bundles.py
python src/nml_hand_exo/robotics/test_policy_profiles.py
python src/nml_hand_exo/robotics/test_robot_config.py
python src/nml_hand_exo/robotics/test_repo_manifests.py
python src/nml_hand_exo/robotics/test_adapter_conformance.py
```

If NeuroBridge runtime is unavailable, runtime-coupled tests may skip by design.

## Suggested Commit Sequence

```bash
git add docs/PHASE2_CUTOVER_CHECKLIST.md docs/NEUROBRIDGE_BOUNDARY.md
git commit -m "docs: add phase-2 cutover checklist and boundary status"

git add src/nml_hand_exo/robotics/ai_runtime.py src/nml_hand_exo/robotics/bridge.py src/nml_hand_exo/robotics/robot_orchestrator.py src/nml_hand_exo/robotics/adapters/virtual_openclaw_adapter.py
git commit -m "refactor(robotics): emit transitional phase-2 ownership warnings"
```