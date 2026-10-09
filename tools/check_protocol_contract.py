"""Compare literal host commands with firmware parser command branches."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re
import sys


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


def protobuf_routes(root: Path) -> dict[str, str]:
    """Return the declared route for every known firmware text command."""
    source = root / "src"
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    from nml_hand_exo.interface._protocol_routes import protobuf_command_route

    commands = firmware_commands(root / "src/cpp/nml_hand_exo/utils.cpp")
    return {command: protobuf_command_route(command) for command in commands}


def protobuf_schema_commands(path: Path) -> set[str]:
    source = path.read_text(encoding="utf-8")
    match = re.search(r"\benum\s+Command\s*\{(.*?)\}", source, re.DOTALL)
    if match is None:
        return set()
    return {
        name.lower()
        for name in re.findall(
            r"^\s*([A-Z][A-Z0-9_]*)\s*=\s*\d+\s*;",
            match.group(1),
            re.MULTILINE,
        )
    }


def protobuf_contract_errors(root: Path) -> list[str]:
    """Cross-check parser routes, schema opcodes and C++ dispatch symbols."""
    source = root / "src"
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    from nml_hand_exo.interface._protocol_routes import (
        PROTOBUF_ONLY_COMMANDS,
        PROTOBUF_PRIMARY_TEXT_COMMANDS,
        PROTOBUF_RPC_COMMANDS,
    )

    parser_commands = firmware_commands(root / "src/cpp/nml_hand_exo/utils.cpp")
    declared_parser = PROTOBUF_RPC_COMMANDS | PROTOBUF_PRIMARY_TEXT_COMMANDS
    errors = []
    if parser_commands != declared_parser:
        errors.append(
            "firmware/route mismatch: missing="
            f"{sorted(parser_commands - declared_parser)}, extra="
            f"{sorted(declared_parser - parser_commands)}"
        )
    schema = protobuf_schema_commands(root / "protocol/exo_usb.proto")
    expected_schema = PROTOBUF_RPC_COMMANDS | PROTOBUF_ONLY_COMMANDS
    if schema != expected_schema:
        errors.append(
            "schema/route mismatch: missing="
            f"{sorted(expected_schema - schema)}, extra={sorted(schema - expected_schema)}"
        )
    dispatch = (root / "src/cpp/nml_hand_exo/protobuf_usb.cpp").read_text(
        encoding="utf-8"
    )
    missing_dispatch = sorted(
        command
        for command in expected_schema
        if f"exo_usb_Command_{command.upper()}" not in dispatch
    )
    if missing_dispatch:
        errors.append(f"protobuf_usb.cpp lacks dispatch symbols: {missing_dispatch}")
    return errors


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
    parser.add_argument(
        "--matrix", action="store_true",
        help="print every firmware command and its Protobuf-build route",
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
    contract_errors = protobuf_contract_errors(root)
    if contract_errors:
        for error in contract_errors:
            print(error)
        return 1
    try:
        routes = protobuf_routes(root)
    except KeyError as exc:
        print(str(exc))
        return 1
    counts = {
        route: sum(value == route for value in routes.values())
        for route in sorted(set(routes.values()))
    }
    if args.matrix:
        for command in sorted(routes):
            print(f"{command:28} {routes[command]}")
    summary = ", ".join(f"{route}={count}" for route, count in counts.items())
    print(f"Protocol contract OK ({summary})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
