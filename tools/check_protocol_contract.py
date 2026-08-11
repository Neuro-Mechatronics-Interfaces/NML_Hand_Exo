"""Compare literal host commands with firmware parser command branches."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re


def _literal_command(expr: ast.expr) -> str | None:
    text = None
    if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
        text = expr.value
    elif isinstance(expr, ast.JoinedStr) and expr.values:
        first = expr.values[0]
        if isinstance(first, ast.Constant) and isinstance(first.value, str):
            text = first.value
    if not text:
        return None
    command = text.strip().split(":", 1)[0]
    return command if command and "{" not in command else None


def host_commands(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    commands: set[str] = set()
    assigned_commands: dict[str, set[str]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        command = _literal_command(node.value)
        if command:
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assigned_commands.setdefault(target.id, set()).add(command)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr in {"send_command", "_command_transaction"}:
            command = _literal_command(node.args[0])
            if command is None and isinstance(node.args[0], ast.Name):
                commands.update(assigned_commands.get(node.args[0].id, set()))
        elif node.func.attr == "_get_motor_attribute":
            command_expr = next(
                (keyword.value for keyword in node.keywords if keyword.arg == "command"),
                None,
            )
            if command_expr is None and len(node.args) >= 4:
                command_expr = node.args[3]
            command = _literal_command(command_expr) if command_expr else None
            if command is None and isinstance(node.args[0], ast.Constant):
                command = f"get_{node.args[0].value}"
        else:
            continue
        if command:
            commands.add(command)
    return commands


def firmware_commands(path: Path) -> set[str]:
    source = path.read_text(encoding="utf-8")
    commands = set(re.findall(r'\bcmd\s*==\s*"([^"]+)"', source))
    # A few legacy commands compare the unsplit token directly (for example
    # ``token == "help"`` and ``token == "oled:on"``).
    commands.update(
        value.split(":", 1)[0]
        for value in re.findall(r'\btoken\s*==\s*"([^"]+)"', source)
    )
    return commands


def missing_host_commands(host_path: Path, firmware_path: Path) -> set[str]:
    return host_commands(host_path) - firmware_commands(firmware_path)


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host", type=Path,
        default=root / "src/nml_hand_exo/interface/_hand_exo.py",
    )
    parser.add_argument(
        "--firmware", type=Path,
        default=root / "src/cpp/nml_hand_exo/utils.cpp",
    )
    args = parser.parse_args(argv)
    host = host_commands(args.host)
    firmware = firmware_commands(args.firmware)
    missing = sorted(host - firmware)
    print(f"Host literal commands: {len(host)}")
    print(f"Firmware parser commands: {len(firmware)}")
    if missing:
        print("Missing firmware commands: " + ", ".join(missing))
        return 1
    print("Protocol contract OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
