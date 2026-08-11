# Scripts Policy

This directory is intentionally limited to maintained utility scripts.

The former NeuroBridge setup and launch wrappers have been removed. Application
entry points are defined in `pyproject.toml` and installed into the virtual
environment.

## Not Allowed Here

- One-off/manual test scripts
- Experimental probes for temporary debugging
- Scripts that contain core runtime decision logic

## Where Validation Belongs

- Automated tests under `src/nml_hand_exo/robotics/test_*.py`
- CI workflow checks under `.github/workflows/`
- Runtime behavior configured via `config/` manifests and policy modules
