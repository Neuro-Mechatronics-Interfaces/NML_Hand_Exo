from __future__ import annotations

import pytest

import handexo_cli
import nml_task_cue_cli
from nml_hand_exo import __version__


@pytest.mark.parametrize(
    "argv",
    [
        ["--help"],
        ["gui", "--help"],
        ["emg-centroid", "--help"],
        ["emg-intent", "--help"],
    ],
)
def test_handexo_help_does_not_launch_gui(argv, capsys):
    with pytest.raises(SystemExit) as exc_info:
        handexo_cli.main(argv)
    assert exc_info.value.code == 0
    assert "usage:" in capsys.readouterr().out


def test_handexo_version(capsys):
    with pytest.raises(SystemExit) as exc_info:
        handexo_cli.main(["--version"])
    assert exc_info.value.code == 0
    assert __version__ in capsys.readouterr().out


def test_task_cue_help_does_not_launch_gui(capsys):
    with pytest.raises(SystemExit) as exc_info:
        nml_task_cue_cli.main(["--help"])
    assert exc_info.value.code == 0
    assert "usage: nml-task-cue" in capsys.readouterr().out


def test_public_star_import_does_not_require_optional_ml_dependencies():
    namespace: dict[str, object] = {}
    exec("from nml_hand_exo import *", namespace)
    assert "HandExo" in namespace
    assert "SerialComm" in namespace
    assert "ml" not in namespace
