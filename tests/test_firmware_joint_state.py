"""Native behavioral tests; no board, motor, or serial connection is opened."""
from pathlib import Path
import shutil
import subprocess
import re

import pytest

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "src/cpp/nml_hand_exo"


def build_and_run(source, tmp_path):
    compiler = shutil.which("g++") or shutil.which("clang++")
    if not compiler:
        pytest.skip("Native C++ compiler unavailable")
    executable = tmp_path / "joint_state_test.exe"
    build = subprocess.run([compiler, "-std=c++11", "-include", str(FIRMWARE / "io_profile.h"), "-Wall", "-Wextra", "-I", str(FIRMWARE),
                            str(source), "-o", str(executable)], capture_output=True, text=True)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stdout + run.stderr


def test_native_model_and_utc_rollover(tmp_path):
    build_and_run(ROOT / "tests/cpp/joint_state_test.cpp", tmp_path)


def test_exclusive_io_accounting_and_binary_frame_recovery(tmp_path):
    build_and_run(ROOT / "tests/cpp/io_profile_test.cpp", tmp_path)


def test_real_firmware_telemetry_routing(tmp_path):
    # Compile the actual implementation methods with a deterministic bus/clock.
    # This executes the early return and partial-read paths, not source assertions.
    implementation = (FIRMWARE / "nml_hand_exo.cpp").read_text(encoding="utf-8")
    start = implementation.index("bool NMLHandExo::getFastTelemetryRecord(")
    end = implementation.index("bool NMLHandExo::setJointModel(", start)
    declarations = (FIRMWARE / "nml_hand_exo.h").read_text(encoding="utf-8")
    record = declarations[declarations.index("struct __attribute__((packed)) FastTelemetryRecord"):
                          declarations.index("/// @brief One absolute motor target")]
    harness = (ROOT / "tests/cpp/telemetry_harness.cpp").read_text(encoding="utf-8")
    source = tmp_path / "telemetry.cpp"
    source.write_text(harness.replace("// RECORD_DECLARATIONS", record)
                      .replace("// FIRMWARE_IMPLEMENTATION", implementation[start:end]), encoding="utf-8")
    build_and_run(source, tmp_path)


def test_rom_deadline_is_serviced_under_sustained_command_traffic(tmp_path):
    sketch = (FIRMWARE / "nml_hand_exo.ino").read_text(encoding="utf-8")
    loop = sketch[sketch.index("void loop() {"):]
    harness = (ROOT / "tests/cpp/rom_loop_harness.cpp").read_text(encoding="utf-8")
    source = tmp_path / "rom_loop.cpp"
    source.write_text(harness.replace("// FIRMWARE_LOOP", loop), encoding="utf-8")
    build_and_run(source, tmp_path)


def test_usb_nx_stays_on_primary_without_blocking_bluetooth_copy(tmp_path):
    implementation = (FIRMWARE / "utils.cpp").read_text(encoding="utf-8")
    frame = implementation[implementation.index("struct __attribute__((packed)) FastTelemetryHeader"):
                           implementation.index("uint8_t collectFastTelemetryIDs(")]
    send = implementation[implementation.index("void sendFastTelemetry("):
                          implementation.index("void sendFastTelemetryDiag(")]
    harness = (ROOT / "tests/cpp/telemetry_route_harness.cpp").read_text(encoding="utf-8")
    source = tmp_path / "route.cpp"
    source.write_text(harness.replace("// FRAME_DECLARATIONS", frame)
                     .replace("// SEND_TELEMETRY", send), encoding="utf-8")
    build_and_run(source, tmp_path)


def test_real_rom_pulses_adapt_and_stop_excessive_travel(tmp_path):
    implementation = (FIRMWARE / "nml_hand_exo.cpp").read_text(encoding="utf-8")
    config = (FIRMWARE / "config.h").read_text(encoding="utf-8")
    constants = "\n".join(re.findall(r"^constexpr [^\n]*ROM_CAL_[^\n]*;", config, re.MULTILINE))
    start = implementation.index("bool NMLHandExo::romCalReadPosition(")
    end = implementation.index("// ====================================================================================", start)
    writer_start = implementation.index("bool NMLHandExo::writeRomSweepCurrent(")
    writer_end = implementation.index("float NMLHandExo::getGoalCurrent(", writer_start)
    stop_start = implementation.index("void NMLHandExo::stopDirectControl(")
    stop_end = implementation.index("void NMLHandExo::stopAllDirectControl(", stop_start)
    safety_start = implementation.index("void NMLHandExo::serviceDirectControlSafety()")
    safety_end = implementation.index("// Acceleration commands", safety_start)
    harness = (ROOT / "tests/cpp/rom_current_harness.cpp").read_text(encoding="utf-8")
    source = tmp_path / "rom_current.cpp"
    source.write_text(harness.replace("// ROM_CONFIG", constants)
                      .replace("// ROM_IMPLEMENTATION", implementation[start:end] +
                               implementation[implementation.index('void NMLHandExo::romCalBeginReturnHome()'):
                                              implementation.index('void NMLHandExo::romCalFinish()')])
                      .replace("// ROM_WRITER", implementation[writer_start:writer_end])
                      .replace("// ROM_STOP", implementation[stop_start:stop_end])
                      .replace("// ROM_SAFETY", implementation[safety_start:safety_end]), encoding="utf-8")
    build_and_run(source, tmp_path)
