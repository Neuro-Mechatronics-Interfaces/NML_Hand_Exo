# NML Hand Exo - Documentation Guide

This folder contains the **Sphinx documentation** for the NML Hand Exoskeleton project.

## 📚 What's Inside

- `source/` - Documentation source files (RST format)
  - `index.rst` - Main documentation page
  - `quickstart.rst` - Getting started guide
  - `python_api.rst` - Python API reference
  - `cpp_api.rst` - C++ firmware API reference
  - `usage.rst` - Usage examples
  - `faq.rst` - Frequently asked questions
  - `conf.py` - Sphinx configuration
  
- `build/` - Generated documentation (HTML)
  - `html/` - Built website files
  - `doxygen/` - C++ API docs from Doxygen

- `Doxyfile` - Doxygen configuration for C++ docs
- `Makefile` / `make.bat` - Build commands

## 🛠️ Building Documentation Locally

### Quick Start

**Option 1: PowerShell Script (Recommended)**
```powershell
cd docs
.\build_docs.ps1
```

**Option 2: Manual Build**
```powershell
cd docs
.\make.bat html
Start-Process "build\html\index.html"
```

### Prerequisites

Install documentation tools:
```powershell
# Python packages
pip install sphinx sphinx-rtd-theme breathe sphinx-autodoc-typehints myst-parser sphinx-copybutton m2r2

# Doxygen (for C++ API docs) - Optional
# Download from: https://www.doxygen.nl/download.html
# Or install with: choco install doxygen
```

### Live Development Server

For automatic rebuilding when you edit docs:

```powershell
cd docs
.\serve_docs.ps1

# Or manually:
pip install sphinx-autobuild
sphinx-autobuild source build/html --open-browser
```

This will:
- Start a local web server at http://127.0.0.1:8000
- Open your browser automatically
- Watch for file changes
- Rebuild and refresh automatically

## 📝 Editing Documentation

Documentation is written in **reStructuredText (RST)** format.

### Common RST Syntax

**Headings:**
```rst
Main Title
==========

Section
-------

Subsection
^^^^^^^^^^
```

**Code Blocks:**
```rst
.. code-block:: python

   from nml_hand_exo.interface import HandExo
   exo = HandExo()
```

**Links:**
```rst
`Link text <https://example.com>`_
```

**Lists:**
```rst
* Item 1
* Item 2
  
1. Numbered item
2. Another item
```

**Notes/Warnings:**
```rst
.. note::
   This is a note.

.. warning::
   This is a warning.
```

### Adding New Pages

1. Create a new `.rst` file in `source/`
2. Add it to the table of contents in `index.rst`:

```rst
.. toctree::
   :maxdepth: 2
   
   quickstart
   your_new_page
   python_api
```

## 🌐 Online Documentation

Docs are automatically built and deployed to GitHub Pages when you push to `main`.

View at: https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/

## 📖 Sphinx Resources

- [Sphinx Documentation](https://www.sphinx-doc.org/)
- [RST Primer](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html)
- [Read the Docs Theme](https://sphinx-rtd-theme.readthedocs.io/)
- [Doxygen Manual](https://www.doxygen.nl/manual/)

## 🐛 Troubleshooting

**Build fails with import errors:**
- Make sure the package is installed: `pip install -e .` from project root
- Check `sys.path` in `source/conf.py`

**Doxygen not found:**
- Optional for Python-only docs
- Required for C++ API documentation
- Install from https://www.doxygen.nl/download.html

**Changes not showing:**
- Clean build: `.\make.bat clean` then `.\make.bat html`
- Check for syntax errors in RST files
- Restart live server if using `sphinx-autobuild`

**Theme looks wrong:**
- Install theme: `pip install sphinx-rtd-theme`
- Check `html_theme` in `source/conf.py`

## 🚀 Quick Commands

```powershell
# Build HTML docs
.\make.bat html

# Clean build directory
.\make.bat clean

# Build and serve with live reload
.\serve_docs.ps1

# Check for broken links
.\make.bat linkcheck

# View built docs
Start-Process "build\html\index.html"
```
