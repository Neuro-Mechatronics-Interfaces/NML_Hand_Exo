# Jekyll Documentation - Setup Guide

This folder contains the **Jekyll-based marketing website** using the Minimal Mistakes theme.

## 🌐 Two Documentation Systems

This project has TWO documentation systems:

1. **Jekyll Site** (this folder) - Marketing, tutorials, assembly guides
   - Theme: Minimal Mistakes
   - URL: Root of GitHub Pages (/)
   - Content: User-facing docs, getting started, tutorials

2. **Sphinx/Doxygen API** (in `../../docs/`) - API reference documentation
   - Tools: Sphinx (Python) + Doxygen (C++)
   - URL: `/api/` on GitHub Pages
   - Content: API reference, function documentation

## 📋 Prerequisites

### Install Ruby

**Windows:**
```powershell
# Option 1: Ruby Installer (Recommended)
# Download from: https://rubyinstaller.org/
# Choose Ruby+Devkit version

# Option 2: Chocolatey
choco install ruby

# After installation, restart your terminal
```

**Verify Installation:**
```powershell
ruby --version
gem --version
```

### Install Bundler

```powershell
gem install bundler
```

## 🚀 Quick Start

### Serve Locally (with Live Reload)

```powershell
cd exo_docs_scaffold\website_jekyll
.\serve_jekyll.ps1
```

This will:
- Install all dependencies (first run only)
- Start Jekyll server at http://localhost:4000
- Open browser automatically
- Auto-rebuild on file changes
- Live reload pages

### Build Static Site

```powershell
cd exo_docs_scaffold\website_jekyll
.\build_jekyll.ps1
```

Generates static HTML in `_site/` folder.

## 📝 Manual Commands

```powershell
# Install dependencies
bundle install

# Serve with live reload
bundle exec jekyll serve --livereload

# Build static site
bundle exec jekyll build

# Clean build
bundle exec jekyll clean
```

## 📂 Site Structure

```
website_jekyll/
├── _config.yml          # Jekyll configuration
├── index.md             # Home page
├── quickstart.md        # Getting started guide
├── api.md               # API overview
├── examples.md          # Examples showcase
├── contributing.md      # Contribution guidelines
├── assembly.md          # Hardware assembly
├── python_api.md        # Python API docs
├── community.md         # Community resources
├── papers.md            # Related publications
├── tutorials/           # Tutorial pages
├── docs/                # Additional documentation
├── assets/              # Images, CSS, JS
│   └── images/
├── _data/               # Data files (navigation, authors)
├── _includes/           # Reusable HTML components
├── _sass/               # Custom styles
└── _site/               # Generated site (ignored by git)
```

## ✏️ Editing Content

### Page Front Matter

Every page starts with YAML front matter:

```yaml
---
title: "Page Title"
permalink: /page-name/
layout: single
toc: true
toc_label: "On This Page"
---

# Your content here
```

### Common Layouts

- `single` - Standard page with sidebar
- `splash` - Full-width landing page
- `home` - Home page with recent posts

### Adding Images

```markdown
![Alt text](/assets/images/your-image.jpg)
```

### Code Blocks

```markdown
```python
from nml_hand_exo import HandExo
exo = HandExo()
`` `
```

### Links

```markdown
[Link text]({{ '/page-name/' | relative_url }})
```

### Notices/Callouts

```markdown
**Note:** This is important.
{: .notice--info}

**Warning:** Be careful!
{: .notice--warning}

**Success:** It worked!
{: .notice--success}
```

## 🎨 Customization

### Changing Theme Skin

Edit `_config.yml`:

```yaml
minimal_mistakes_skin: "air"
# Options: default, air, aqua, contrast, dark, dirt, neon, mint, sunrise
```

### Navigation Menu

Edit navigation in `_data/navigation.yml` or `_config.yml`.

### Custom Styles

Add custom CSS in `_sass/custom.scss` or `assets/css/main.scss`.

## 🔧 Troubleshooting

### Ruby Installation Issues (Windows)

If `bundle install` fails:

1. Install Ruby with Devkit: https://rubyinstaller.org/
2. During installation, select "Run 'ridk install'" at the end
3. Choose option 3 (MSYS2 and MINGW development toolchain)
4. Restart terminal after installation

### Port Already in Use

```powershell
# Use different port
bundle exec jekyll serve --port 4001
```

### Dependencies Not Installing

```powershell
# Update Bundler
gem update bundler

# Clean and reinstall
bundle clean --force
bundle install
```

### Changes Not Showing

```powershell
# Hard rebuild
bundle exec jekyll clean
bundle exec jekyll serve --livereload
```

### Minimal Mistakes Theme Issues

```powershell
# Update theme
bundle update
```

## 🌐 GitHub Pages Deployment

This site can be deployed to GitHub Pages automatically.

### Setup

1. Push to `main` branch
2. Enable GitHub Pages in repository settings
3. Choose source: GitHub Actions (if using workflow) or branch

### Workflow

The `.github/workflows/` folder should contain a workflow that:
- Builds Jekyll site
- Builds Doxygen C++ docs
- Deploys both to GitHub Pages

## 📚 Resources

- [Jekyll Documentation](https://jekyllrb.com/docs/)
- [Minimal Mistakes Theme](https://mmistakes.github.io/minimal-mistakes/)
- [GitHub Pages with Jekyll](https://docs.github.com/en/pages/setting-up-a-github-pages-site-with-jekyll)
- [Kramdown Syntax](https://kramdown.gettalong.org/quickref.html)

## 💡 Tips

**Live Reload:**
- Use `--livereload` flag for instant preview of changes
- Browser auto-refreshes on save

**Incremental Builds:**
- Use `--incremental` for faster rebuilds (experimental)

**Draft Posts:**
- Create in `_drafts/` folder
- Serve with `--drafts` to preview

**Custom Domains:**
- Add `CNAME` file with your domain
- Configure DNS settings

## 🔗 Integration with Sphinx Docs

The Jekyll site and Sphinx API docs are separate but linked:

- Jekyll site: Marketing, tutorials, getting started
- Sphinx docs: Technical API reference

Link between them:
```markdown
See the [Python API Reference]({{ '/api/python/' | relative_url }})
```

Both are deployed to GitHub Pages:
- Jekyll: Root URL (/)
- Sphinx: /api/ subdirectory
