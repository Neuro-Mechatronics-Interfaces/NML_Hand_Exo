# Safety Requirements Trace (Research-Grade)

This document defines lightweight, testable safety requirements for NML_Hand_Exo.

## Scope

- Robotics planning and execution boundary in `src/nml_hand_exo/robotics`.
- Runtime configuration via `config/` manifests.
- No direct claim of medical-device certification; this is process discipline for safer research workflows.

## Requirements

| ID | Requirement | Enforcement Location | Verification |
| --- | --- | --- | --- |
| SAF-001 | Unsupported joints must be rejected before execution. | `src/nml_hand_exo/robotics/policy.py` (`JointSupportRule`) | `src/nml_hand_exo/robotics/test_robotics_platform.py` |
| SAF-002 | Relative and absolute limits must be checked in software policy. | `src/nml_hand_exo/robotics/policy.py` (`JointLimitRule`) | `src/nml_hand_exo/robotics/test_robotics_platform.py` |
| SAF-003 | Confirmation rules must be deterministic and mode-controlled. | `src/nml_hand_exo/robotics/policy.py` (`ConfirmationRule`) | `src/nml_hand_exo/robotics/test_robot_config.py` + policy unit assertions |
| SAF-004 | Adapter compatibility/health checks must gate adapter use. | `src/nml_hand_exo/robotics/registry.py` | `src/nml_hand_exo/robotics/test_adapter_conformance.py` |
| SAF-005 | Deployment bundle manifests must be schema-validated. | `src/nml_hand_exo/robotics/bundles.py` | `src/nml_hand_exo/robotics/test_bundles.py`, `src/nml_hand_exo/robotics/test_repo_manifests.py` |
| SAF-006 | Policy profile manifests must be schema-validated. | `src/nml_hand_exo/robotics/policy_profiles.py` | `src/nml_hand_exo/robotics/test_policy_profiles.py`, `src/nml_hand_exo/robotics/test_repo_manifests.py` |
| SAF-007 | Robot adapter config manifests must be schema-validated and matched to adapter IDs. | `src/nml_hand_exo/robotics/config.py` | `src/nml_hand_exo/robotics/test_robot_config.py`, `src/nml_hand_exo/robotics/test_repo_manifests.py` |
| SAF-008 | Runtime setup must default to pinned submodule sources, not machine-local paths. | `scripts/setup_neurobridge_submodule.bat`, `scripts/setup_ai_submodule_env.py`, `.gitmodules` | Code review + CI on committed files |

## Process Rules

1. Every new safety-relevant behavior must add or reference a `SAF-*` requirement.
2. Every `SAF-*` requirement must map to at least one automated test.
3. Runtime behavior changes should be made through config or policy modules, not ad-hoc scripts.
