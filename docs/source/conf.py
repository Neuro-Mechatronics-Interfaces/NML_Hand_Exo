"""Sphinx configuration for the NML Hand Exoskeleton documentation."""

from __future__ import annotations

import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from nml_hand_exo import __version__

project = "NML Hand Exoskeleton"
copyright = "2026, Neuromechatronics Lab"
author = "Neuromechatronics Lab"
version = release = __version__

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx_copybutton",
]

templates_path = ["_templates"]
exclude_patterns: list[str] = []
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
autodoc_typehints = "description"
autodoc_member_order = "bysource"
add_module_names = False

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_logo = "_static/LabLogoRedSquare.png"
html_css_files = ["custom.css"]
html_theme_options = {
    "style_nav_header_background": "#252526",
    "navigation_depth": 4,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "includehidden": True,
    "titles_only": False,
}
html_context = {
    "display_github": True,
    "github_user": "Neuro-Mechatronics-Interfaces",
    "github_repo": "NML_Hand_Exo",
    "github_version": "main",
    "conf_py_path": "/docs/source/",
}
