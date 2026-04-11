# Scripts Policy

This directory is intentionally limited to thin wrappers and environment bootstrap.

## Allowed Scripts

- Setup/bootstrap wrappers:
  - `setup_neurobridge_submodule.bat`
  - `setup_ai_submodule_env.py`
- Runtime launch wrappers:
  - `run_ai_agent.bat`
  - `run_ai_agent.py`
  - `run_ai_assist_gui.bat`
  - `run_ai_assist_gui.py`
  - `run_exo_visualizer.bat`
  - `run_exo_visualizer.py`

## Not Allowed Here

- One-off/manual test scripts
- Experimental probes for temporary debugging
- Scripts that contain core runtime decision logic

## Where Validation Belongs

- Automated tests under `src/nml_hand_exo/robotics/test_*.py`
- CI workflow checks under `.github/workflows/`
- Runtime behavior configured via `config/` manifests and policy modules
