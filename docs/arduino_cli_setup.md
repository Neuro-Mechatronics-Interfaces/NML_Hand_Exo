# Setting Up Arduino Command-Line Interface #

This guide installs `arduino-cli` and the known board/library dependencies for the `nml_hand_exo` firmware on:

- Windows 11
- macOS on Apple Silicon, including MacBook Pro M4

The firmware target is the ROBOTIS OpenRB-150.

> Note: commands below assume the project already contains the generated nanopb protocol files such as `protocol/exo_usb.pb.c` and `protocol/exo_usb.pb.h`.

---

## 1. Windows 11

Open **PowerShell**.

### 1.1 Install Arduino CLI

Preferred:

```powershell
winget install --id ArduinoSA.CLI -e --source winget
```

Close and reopen PowerShell, then verify:

```powershell
arduino-cli version
```

If the WinGet package is unavailable, Arduino also publishes a current 64-bit MSI:

```powershell
$msi = "$env:TEMP\arduino-cli.msi"
Invoke-WebRequest `
  -Uri "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.msi" `
  -OutFile $msi

Start-Process msiexec.exe -Wait -ArgumentList "/i `"$msi`""
```

Then reopen PowerShell and verify:

```powershell
arduino-cli version
```

### 1.2 Initialize Arduino CLI

```powershell
arduino-cli config init
```

If a configuration file already exists, that is fine; do not overwrite a working configuration unnecessarily.

### 1.3 Install OpenRB-150 board support

Add the ROBOTIS package index:

```powershell
arduino-cli config add board_manager.additional_urls "https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json"
```

Update package indexes:

```powershell
arduino-cli core update-index
```

Install the Arduino SAMD dependency and OpenRB-150 core:

```powershell
arduino-cli core install arduino:samd
arduino-cli core install OpenRB-150:samd
```

Verify:

```powershell
arduino-cli core list
arduino-cli board listall | Select-String -Pattern "OpenRB"
```

The OpenRB-150 FQBN should be:

```text
OpenRB-150:samd:OpenRB-150
```

### 1.4 Install Arduino libraries

Update the library index:

```powershell
arduino-cli lib update-index
```

Install the known `nml_hand_exo` dependencies:

```powershell
arduino-cli lib install "Dynamixel2Arduino"
arduino-cli lib install "Adafruit BNO055"
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit BusIO"
```

Arduino CLI normally installs declared transitive dependencies automatically, but explicitly installing the Adafruit support libraries makes the environment more reproducible.

Verify:

```powershell
arduino-cli lib list
```

### 1.5 Install nanopb generator tooling

The nanopb **generator** runs on the host computer. The nanopb **runtime** (`pb*.c` / `pb*.h`) is compiled into the firmware.

Install Python 3.12 if needed:

```powershell
winget install --id Python.Python.3.12 -e
```

Close and reopen PowerShell, then create a dedicated nanopb virtual environment:

```powershell
py -3.12 -m venv "$env:USERPROFILE\.venvs\nanopb"
```

Activate it:

```powershell
& "$env:USERPROFILE\.venvs\nanopb\Scripts\Activate.ps1"
```

Install nanopb and its protobuf tooling:

```powershell
python -m pip install --upgrade pip
python -m pip install "nanopb==0.4.9.1" protobuf grpcio-tools
```

Verify:

```powershell
nanopb_generator --version
```

If PowerShell blocks virtual-environment activation, the tooling can still be invoked directly:

```powershell
& "$env:USERPROFILE\.venvs\nanopb\Scripts\nanopb_generator.exe" --version
```

---

## 2. macOS — MacBook Pro M4 / Apple Silicon

Open **Terminal**.

### 2.1 Install Homebrew if needed

Check first:

```bash
brew --version
```

If Homebrew is not installed:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

On Apple Silicon, Homebrew normally installs under `/opt/homebrew`.

If `brew` is not immediately found after installation:

```bash
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
```

Verify:

```bash
brew --version
uname -m
```

On an M4 Mac, `uname -m` should report:

```text
arm64
```

### 2.2 Install Arduino CLI

```bash
brew update
brew install arduino-cli
```

Verify:

```bash
arduino-cli version
```

Homebrew installs the native Apple Silicon build on an M4 Mac.

### 2.3 Initialize Arduino CLI

```bash
arduino-cli config init
```

If a configuration file already exists, leave the existing working configuration in place.

### 2.4 Install OpenRB-150 board support

Add the ROBOTIS package index:

```bash
arduino-cli config add board_manager.additional_urls \
  "https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json"
```

Update package indexes:

```bash
arduino-cli core update-index
```

Install the Arduino SAMD dependency and OpenRB-150 core:

```bash
arduino-cli core install arduino:samd
arduino-cli core install OpenRB-150:samd
```

Verify:

```bash
arduino-cli core list
arduino-cli board listall | grep -i openrb
```

The OpenRB-150 FQBN should be:

```text
OpenRB-150:samd:OpenRB-150
```

### 2.5 Install Arduino libraries

```bash
arduino-cli lib update-index
```

Install the known `nml_hand_exo` dependencies:

```bash
arduino-cli lib install "Dynamixel2Arduino"
arduino-cli lib install "Adafruit BNO055"
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SSD1306"
arduino-cli lib install "Adafruit BusIO"
```

Verify:

```bash
arduino-cli lib list
```

### 2.6 Install nanopb generator tooling

Install Python and create an isolated nanopb environment:

```bash
brew install python@3.12
```

Create the environment:

```bash
/opt/homebrew/bin/python3.12 -m venv ~/.venvs/nanopb
```

Activate it:

```bash
source ~/.venvs/nanopb/bin/activate
```

Install nanopb and protobuf tooling:

```bash
python -m pip install --upgrade pip
python -m pip install "nanopb==0.4.9.1" protobuf grpcio-tools
```

Verify:

```bash
nanopb_generator --version
```

Outside the activated environment, invoke it directly with:

```bash
~/.venvs/nanopb/bin/nanopb_generator --version
```

---

## 3. Nanopb runtime files used by the firmware

Installing the Python package provides the code generator, but the OpenRB firmware also needs the nanopb C runtime.

The firmware should compile these runtime files:

```text
pb.h
pb_common.h
pb_common.c
pb_encode.h
pb_encode.c
pb_decode.h
pb_decode.c
```

along with the generated project files:

```text
protocol/exo_usb.pb.h
protocol/exo_usb.pb.c
```

A typical project layout is:

```text
nml_hand_exo/
├── protocol/
│   ├── exo_usb.pb.c
│   └── exo_usb.pb.h
├── nanopb/
│   ├── pb.h
│   ├── pb_common.c
│   ├── pb_common.h
│   ├── pb_encode.c
│   ├── pb_encode.h
│   ├── pb_decode.c
│   └── pb_decode.h
├── AxonEnumProtocol.h
├── AxonUsbPeripheral.cpp
├── AxonUsbPeripheral.h
├── ...
└── nml_hand_exo.ino
```

For reproducible builds, vendor a pinned nanopb runtime version in the repository rather than depending on a machine-wide Arduino wrapper library.

The current stable nanopb release used in this guide is:

```text
0.4.9.1
```

If the repository already vendors `pb*.c` and `pb*.h`, do **not** install a second nanopb runtime.

`nanopb-arduino` is not required merely to use `pb_encode()` / `pb_decode()`; it is a separate Arduino `Stream` adapter.

---

## 4. Verify the complete toolchain

### Windows

```powershell
arduino-cli version
arduino-cli core list
arduino-cli lib list
arduino-cli board list
```

If the nanopb virtual environment is active:

```powershell
nanopb_generator --version
```

### macOS

```bash
arduino-cli version
arduino-cli core list
arduino-cli lib list
arduino-cli board list
```

If the nanopb virtual environment is active:

```bash
nanopb_generator --version
```

---

## 5. Detect the OpenRB-150 serial port

Connect the OpenRB-150 by USB.

### Windows

```powershell
arduino-cli board list
```

Typical port:

```text
COM3
COM4
COM5
...
```

For scripts:

```powershell
$env:COM = "COM5"
```

Then:

```powershell
arduino-cli board list
Write-Host $env:COM
```

### macOS

```bash
arduino-cli board list
```

Typical device names include:

```text
/dev/cu.usbmodem...
```

For scripts:

```bash
export COM=/dev/cu.usbmodem1101
```

Then:

```bash
echo "$COM"
```

Using `/dev/cu.*` is generally preferable to `/dev/tty.*` for an application initiating the serial connection.

---

## 6. Test compilation

From the directory containing `nml_hand_exo.ino`:

### Windows PowerShell

```powershell
arduino-cli compile `
  --fqbn "OpenRB-150:samd:OpenRB-150" `
  .
```

### macOS

```bash
arduino-cli compile \
  --fqbn "OpenRB-150:samd:OpenRB-150" \
  .
```

If this succeeds, the board core and compile-time library dependencies are installed correctly.

---

## 7. Upload

After determining the port:

### Windows

```powershell
$env:COM = "COM5"

arduino-cli upload `
  -p $env:COM `
  --fqbn "OpenRB-150:samd:OpenRB-150" `
  .
```

### macOS

```bash
export COM=/dev/cu.usbmodem1101

arduino-cli upload \
  -p "$COM" \
  --fqbn "OpenRB-150:samd:OpenRB-150" \
  .
```

If OpenRB upload fails, ROBOTIS recommends entering bootloader mode by double-pressing the board's reset button and trying the upload again.

---

## 8. Useful dependency audit

To inspect the project's `#include` statements and catch additional third-party libraries:

### Windows PowerShell

```powershell
Get-ChildItem -Recurse -Include *.h,*.hpp,*.c,*.cpp,*.ino |
  Select-String '^\s*#include\s*[<"][^>"]+[>"]' |
  ForEach-Object { $_.Matches.Value } |
  Sort-Object -Unique
```

### macOS

```bash
grep -RhoE '^[[:space:]]*#include[[:space:]]*[<"][^>"]+[>"]' . \
  --include='*.h' \
  --include='*.hpp' \
  --include='*.c' \
  --include='*.cpp' \
  --include='*.ino' \
  | sort -u
```

This is useful if `oled.cpp` or another source file gains a new dependency later.

---

## References

- Arduino CLI installation: https://docs.arduino.cc/arduino-cli/installation
- Arduino CLI library installation: https://docs.arduino.cc/arduino-cli/commands-reference/arduino-cli_lib_install
- ROBOTIS OpenRB-150 documentation: https://emanual.robotis.com/docs/en/parts/controller/openrb-150/
- ROBOTIS OpenRB-150 package index: https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json
- Dynamixel2Arduino: https://github.com/ROBOTIS-GIT/Dynamixel2Arduino
- nanopb: https://github.com/nanopb/nanopb
- nanopb PyPI package: https://pypi.org/project/nanopb/
