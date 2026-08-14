from __future__ import annotations

import re
from pathlib import Path

import nml_hand_exo


ROOT = Path(__file__).resolve().parents[1]


def test_release_version_is_consistent():
    pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    citation = (ROOT / "CITATION.cff").read_text(encoding="utf-8")
    project_match = re.search(r'^version\s*=\s*"([^"]+)"', pyproject, re.MULTILINE)
    citation_match = re.search(
        r'^version:\s*["\']?([^"\'\s]+)', citation, re.MULTILINE
    )

    assert project_match is not None
    assert citation_match is not None
    assert project_match.group(1) == nml_hand_exo.__version__
    assert citation_match.group(1) == nml_hand_exo.__version__


def test_console_entry_points_target_maintained_modules():
    pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert 'handexo = "nml_hand_exo.cli:main"' in pyproject
    assert 'nml-task-cue = "nml_task_cue_cli:main"' in pyproject


def test_readme_uses_a_publishable_project_image():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    image = ROOT / "docs" / "assets" / "nml-hand-exo.png"

    assert image.is_file()
    assert image.stat().st_size > 0
    assert "https://raw.githubusercontent.com/" in readme
    assert "/docs/assets/nml-hand-exo.png" in readme
