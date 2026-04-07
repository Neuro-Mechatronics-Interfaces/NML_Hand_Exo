# Phase 1 Migration Checklist (No Breakage)

This checklist implements the boundary split without changing runtime behavior.

## Goal

- Move low-risk, generic prompt assets to NeuroBridge.
- Keep hardware/safety authority in NML_Hand_Exo.
- Preserve current launch and orchestration compatibility.

## Preconditions

- NeuroBridge submodule present at `external/NeuroBridge`.
- Worktree is clean enough to make focused commits.

## Commit 1: Boundary docs only

Purpose: lock decisions before moving files.

```bash
git add docs/NEUROBRIDGE_BOUNDARY.md docs/PHASE1_MIGRATION_CHECKLIST.md
git commit -m "docs: define phase-1 exo vs neurobridge ownership boundary"
```

## Commit 2: Move prompt assets to NeuroBridge

Purpose: relocate generic conversation/prompt engineering assets.

In `NML_Hand_Exo`:

```bash
git rm .github/prompts/agent-prompt-optimizer.prompt.md
git rm .github/prompts/conversation-repair-prompting.prompt.md
git rm .github/prompts/naturalistic-prompt-suite.prompt.md
git commit -m "chore: move generic AI prompt assets to NeuroBridge"
```

In `NeuroBridge` (target path suggestion):

- `prompts/agent-prompt-optimizer.prompt.md`
- `prompts/conversation-repair-prompting.prompt.md`
- `prompts/naturalistic-prompt-suite.prompt.md`

```bash
git add prompts/agent-prompt-optimizer.prompt.md prompts/conversation-repair-prompting.prompt.md prompts/naturalistic-prompt-suite.prompt.md
git commit -m "prompts: add naturalistic/repair/optimizer suites from NML_Hand_Exo"
```

## Commit 3: Transitional annotation (optional, recommended)

Purpose: clearly mark temporary bridge ownership while preserving behavior.

Files to annotate in docstrings/comments only:

- `src/nml_hand_exo/robotics/bridge.py`
- `src/nml_hand_exo/robotics/ai_runtime.py`
- `src/nml_hand_exo/robotics/robot_orchestrator.py`
- `src/nml_hand_exo/robotics/adapters/virtual_openclaw_adapter.py`

Commit:

```bash
git add src/nml_hand_exo/robotics/bridge.py src/nml_hand_exo/robotics/ai_runtime.py src/nml_hand_exo/robotics/robot_orchestrator.py src/nml_hand_exo/robotics/adapters/virtual_openclaw_adapter.py
git commit -m "docs(robotics): mark transitional bridge modules for phase-2 relocation"
```

## Keep in NML_Hand_Exo (Do not move in Phase 1)

- `src/nml_hand_exo/robotics/contracts.py`
- `src/nml_hand_exo/robotics/policy.py`
- `src/nml_hand_exo/robotics/config.py`
- `src/nml_hand_exo/robotics/registry.py`
- `src/nml_hand_exo/robotics/bootstrap.py`
- `src/nml_hand_exo/robotics/adapters/nml_hand_exo_adapter.py`
- `config/robot_adapters/nml_hand_exo.yaml`
- `config/bundles/nml_default.yaml`
- `config/policy_profiles/safe_lab.yaml`
- `docs/SAFETY_REQUIREMENTS.md`

## Validation after each commit

```bash
python src/nml_hand_exo/robotics/test_bundles.py
python src/nml_hand_exo/robotics/test_policy_profiles.py
python src/nml_hand_exo/robotics/test_robot_config.py
python src/nml_hand_exo/robotics/test_repo_manifests.py
```

If NeuroBridge is unavailable locally, runtime-coupled tests may skip by design.
