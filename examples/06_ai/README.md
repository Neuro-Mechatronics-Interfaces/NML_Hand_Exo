# AI Assistant Examples

This folder contains AI assistant entry points that bootstrap the NeuroBridge runtime from the NML_Hand_Exo repository.

## Launch The Assistant GUI

Run from the repository root:

```bash
python examples/06_ai/ai_assist_gui.py
```

On Windows, this launcher detaches by default so the terminal returns immediately. For foreground debugging with terminal logs attached, run:

```bash
python examples/06_ai/ai_assist_gui.py --foreground
```

If the NeuroBridge dependencies are not installed yet, run:

```bash
python scripts/setup_ai_submodule_env.py
```