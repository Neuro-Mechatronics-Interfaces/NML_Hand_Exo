# AI Agent + Visualizer with NeuroBridge Submodule

This guide configures `NML_Hand_Exo` as the main repo and `NeuroBridge` as a pinned dependency submodule for AI runtime.

## Target Topology

- Main repo: `NML_Hand_Exo`
- Submodule: `external/NeuroBridge`
- Runtime source of truth for AI app: `external/NeuroBridge`
- Exo hardware/simulator interface: still from NML Hand Exo package APIs

## 1) Add NeuroBridge as Submodule (one-time)

Fastest path (repo root, in cmd):

```bat
scripts\setup_neurobridge_submodule.bat
```

Manual path (if you prefer step-by-step):

```bat
git submodule add https://github.com/JShulgach/NeuroBridge.git external/NeuroBridge
git submodule update --init --recursive
```

If your remote URL differs, replace it with your fork/internal URL.

## 2) Create and Activate Environment

Use the Python setup script (no activation required):

```bat
python scripts\setup_ai_submodule_env.py
```

Optional (only if you explicitly need editable local NML package installed too):

```bat
python scripts\setup_ai_submodule_env.py --install-local-nml
```

## 3) Install Packages in Correct Order

Install steps are handled by `scripts\setup_ai_submodule_env.py`.

Why this order:
- Ensures AI runtime modules come from NeuroBridge.
- Avoids local package shadowing by default.

## 4) Run AI Agent from NeuroBridge Runtime

`scripts\run_ai_agent.py` defaults to `--bundle nml_default` when no bundle is provided, so both of these are valid:

```bat
scripts\run_ai_agent.bat --command status
scripts\run_ai_agent.bat --bundle nml_default --command status
```

For conversation mode:

```bat
scripts\run_ai_agent.bat
scripts\run_ai_agent.bat --bundle nml_default
```

If you want real motor movement instead of dry-run simulation, pass:

```bat
scripts\run_ai_agent.bat --no-dry-run
```

## 5) Run Exo Visualizer

```bat
scripts\run_exo_visualizer.bat
```

Or invoke Python directly:

```bat
.venv\Scripts\python.exe scripts\run_ai_agent.py --bundle nml_default
.venv\Scripts\python.exe scripts\run_exo_visualizer.py
```

## 6) Typical Two-Terminal Workflow

Terminal A (cmd):

```bat
scripts\run_ai_agent.bat --bundle nml_default
```

Terminal B (cmd):

```bat
scripts\run_exo_visualizer.bat
```

## 7) Submodule Update Workflow

When pulling updates in main repo:

```bat
git submodule update --init --recursive
```

When intentionally bumping NeuroBridge version:

```bat
cd external/NeuroBridge
git fetch --all
git checkout <tag-or-commit>
cd ../..
git add external/NeuroBridge
git commit -m "Pin NeuroBridge to <tag-or-commit>"
```

## Recommended Cleanup Policy

To avoid drift, keep one source of truth for the AI runtime:
- Prefer NeuroBridge submodule code for AI app evolution.
- Keep NML_Hand_Exo focused on core interface/control firmware + docs/examples.
