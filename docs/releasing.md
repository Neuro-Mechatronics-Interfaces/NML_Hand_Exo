# Release workflow

This project uses one version in `pyproject.toml`, `nml_hand_exo.__version__`,
and `CITATION.cff`. The release tag must be the same version prefixed with `v`.

## Prepare and validate

From a clean checkout of the intended release commit:

```powershell
py -3.11 -m venv .venv-release
.\.venv-release\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[analysis,dev]"
python -m pytest -q
python -m compileall -q src examples tools tests
handexo --help
nml-task-cue --help
python -m build
python -m twine check dist/*
```

Inspect both archives before uploading. The wheel must contain only the
supported `nml_hand_exo` package, compatibility launchers, package metadata,
and the license. The sdist additionally contains source tests and build files.

## TestPyPI

Use a TestPyPI token supplied through the prompt or a user-level `.pypirc` that
is outside the repository:

```powershell
python -m twine upload --repository testpypi dist/*
```

Then install the exact version in a fresh environment. The extra PyPI index is
needed because project dependencies generally come from production PyPI:

```powershell
py -3.11 -m venv .venv-testpypi
.\.venv-testpypi\Scripts\Activate.ps1
python -m pip install --index-url https://test.pypi.org/simple/ `
  --extra-index-url https://pypi.org/simple/ nml-hand-exo==0.2.18
python -c "import nml_hand_exo; print(nml_hand_exo.__version__)"
handexo --help
nml-task-cue --help
```

## Stable PyPI

Only after TestPyPI installation and hardware-independent smoke tests pass:

```powershell
git tag -a v0.2.18 -m "NML Hand Exoskeleton 0.2.18"
git push origin v0.2.18
python -m twine upload dist/*
```

PyPI files and release tags are effectively immutable. Do not reuse a version;
increment it and rebuild if any artifact must change.

## Fresh production installation

```powershell
py -3.11 -m venv .venv-pypi
.\.venv-pypi\Scripts\Activate.ps1
python -m pip install nml-hand-exo==0.2.18
python -c "import nml_hand_exo; print(nml_hand_exo.__version__)"
handexo --help
nml-task-cue --help
```
