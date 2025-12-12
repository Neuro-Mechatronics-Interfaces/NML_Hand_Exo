---
title: Quickstart
permalink: /quickstart/
layout: single
toc: true
toc_sticky: true
---

Welcome! This guide gets you from **clone → flash → first motion** on Windows (and notes for macOS/Linux).

> If you haven’t assembled the hardware yet, start with the [Assembly guide](/assembly/).

---

## 1) Prerequisites

### On Windows 11
- **Python 3.10+** (add to PATH)
- **Git**
- Optional: **VS Code** (recommended)
- For firmware:
  - **PlatformIO Core** (`pip install platformio`) *or* **Arduino CLI**
- USB driver (Windows): CDC/WinUSB (use **Zadig** if needed)

### macOS / Linux (notes)
- Install Python 3.10+, Git, and PlatformIO/Arduino CLI.
- On Linux, add your user to the `dialout`/`uucp` group for serial ports:
  ```bash
  sudo usermod -a -G dialout $USER
  # log out/in after changing groups
