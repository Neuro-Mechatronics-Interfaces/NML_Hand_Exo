# Documentation

The maintained documentation has two parts:

- Architecture and protocol notes in this directory (`*.md`).
- The Sphinx reference under `docs/source/`, published to GitHub Pages.

Build the reference from the repository root:

```powershell
python -m pip install -e ".[docs]"
python -m sphinx -W --keep-going docs/source docs/build/html
```

Sphinx writes generated output under `docs/build/`, which is intentionally
ignored by git.

When changing firmware commands, update `serial_protocol.md`, the Python
parser, relevant examples, and regression tests together. When changing GUI
workflows, update `gui_workflow.md` and any linked architecture note.
