"""Stateful OpenRB serial-protocol emulator used by integration tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class ReplyFault:
    """Override the next matching reply or transport behavior."""

    command: str | None = None
    response: str | None = None
    delay_s: float = 0.0
    error: Exception | None = None


class FakeOpenRBComm:
    """Small stateful stand-in for the firmware's delimited command channel.

    It deliberately implements the same ``send``/``receive`` surface as
    ``SerialComm``. Faults can inject empty, malformed, delayed, unrelated, or
    exceptional replies without changing application code.
    """

    def __init__(
        self,
        *,
        motor_ids: tuple[int, ...] = tuple(range(11, 20)),
        version: str = "0.6.2",
        delimiter: str = "\n",
    ):
        self.verbose = False
        self.response_timeout = 0.75
        self.version = str(version)
        self.delimiter = delimiter
        self.motor_ids = tuple(int(mid) for mid in motor_ids)
        self.names = {
            mid: name
            for mid, name in zip(
                self.motor_ids,
                ("wrist", "wrist2", "thumbadd", "thumbrot", "thumbflex",
                 "index", "middle", "ring", "pinky"),
            )
        }
        self.angles = {mid: 0.0 for mid in self.motor_ids}
        self.absolute_angles = {mid: 0.0 for mid in self.motor_ids}
        self.currents = {mid: 0.0 for mid in self.motor_ids}
        self.current_limits = {mid: 910 for mid in self.motor_ids}
        self.velocity_limits_raw = {mid: 44 for mid in self.motor_ids}
        self.total_current_budget_mA = 800
        self.enabled = {mid: False for mid in self.motor_ids}
        self.limits = {mid: (-90.0, 90.0) for mid in self.motor_ids}
        self.holds: dict[int, float] = {}
        self.control_mode = "position"
        self.sent: list[str] = []
        self._replies: deque[str] = deque()
        self._faults: deque[ReplyFault] = deque()
        self._connected = False
        self._lock = threading.Lock()

    def connect(self):
        self._connected = True

    def close(self):
        self._connected = False

    def is_connected(self):
        return self._connected

    def flush_input(self):
        self._replies.clear()

    def queue_fault(self, fault: ReplyFault):
        self._faults.append(fault)

    def send(self, message: str):
        if not self._connected:
            raise ConnectionError("Fake OpenRB is disconnected")
        command = str(message).strip().rstrip(";\r\n")
        with self._lock:
            self.sent.append(command)
            self._replies.append(self._dispatch(command))

    def receive(self, wait_until_return=False, timeout=None):
        del wait_until_return
        with self._lock:
            command = self.sent[-1] if self.sent else ""
            fault = self._pop_fault(command)
            reply = self._replies.popleft() if self._replies else ""
        if fault is not None:
            if fault.delay_s:
                effective_timeout = (
                    float(timeout) if timeout is not None else self.response_timeout
                )
                limit = effective_timeout
                time.sleep(min(float(fault.delay_s), limit))
                if fault.delay_s > effective_timeout:
                    return ""
            if fault.error is not None:
                raise fault.error
            if fault.response is not None:
                return fault.response
        return reply

    def _pop_fault(self, command: str) -> ReplyFault | None:
        for index, fault in enumerate(self._faults):
            if fault.command is None or command.startswith(fault.command):
                del self._faults[index]
                return fault
        return None

    def _motor_line(self, motor_id: int, field: str, value) -> str:
        name = self.names.get(motor_id, f"motor_{motor_id}")
        return f"Motor: {{name: {name}, id: {motor_id}, {field}: {value}}}"

    def _motor_block(self, field: str, values: dict[int, object]) -> str:
        return "\n".join(
            f"Motor {index}: {{name: {self.names.get(mid, f'motor_{mid}')}, "
            f"id: {mid}, {field}: {values[mid]}}}"
            for index, mid in enumerate(self.motor_ids)
        )

    def _target_ids(self, token: str) -> tuple[int, ...]:
        if token.lower() == "all":
            return self.motor_ids
        motor_id = int(token)
        return (motor_id,) if motor_id in self.motor_ids else ()

    def _dispatch(self, command: str) -> str:
        parts = command.split(":")
        head = parts[0]
        if head == "version":
            return f"Version: {self.version}"
        if head == "info":
            return (
                f"Name: NML_HAND_EXO\nVersion: {self.version}\nSide: right\n"
                f"Number of Motors: {len(self.motor_ids)}"
            )
        if head in {"get_angle", "get_absolute_angle", "get_current", "get_torque", "get_enabled", "get_motor_limits", "get_baud", "get_goal_velocity", "get_goal_acceleration", "get_current_lim"}:
            target = parts[1] if len(parts) > 1 else "all"
            fields = {
                "get_angle": ("angle", self.angles),
                "get_absolute_angle": ("absolute_angle", self.absolute_angles),
                "get_current": ("current", self.currents),
                "get_torque": ("torque", {mid: 0.0 for mid in self.motor_ids}),
                "get_enabled": ("enabled", {mid: str(self.enabled[mid]).lower() for mid in self.motor_ids}),
                "get_motor_limits": ("limits", {mid: f"[{self.limits[mid][0]}, {self.limits[mid][1]}]" for mid in self.motor_ids}),
                "get_baud": ("baudrate", {mid: 1_000_000 for mid in self.motor_ids}),
                "get_goal_velocity": ("velocity", self.velocity_limits_raw),
                "get_goal_acceleration": ("acceleration", {mid: 50 for mid in self.motor_ids}),
                "get_current_lim": ("current_limit", self.current_limits),
            }
            field, values = fields[head]
            ids = self._target_ids(target)
            if not ids:
                return f"ERROR: unknown motor {target}"
            if target.lower() == "all":
                return self._motor_block(field, values)
            return self._motor_line(ids[0], field, values[ids[0]])
        if head in {"enable", "disable"}:
            for mid in self._target_ids(parts[1]):
                self.enabled[mid] = head == "enable"
            return f"OK: {head} {parts[1]}"
        if head == "set_current_lim":
            targets = self._target_ids(parts[1])
            value = min(910, max(1, int(float(parts[2]))))
            for mid in targets:
                self.current_limits[mid] = value
            return f"OK: set_current_lim {parts[1]} {value}"
        if head == "set_goal_velocity":
            targets = self._target_ids(parts[1])
            value = max(1, int(float(parts[2])))
            for mid in targets:
                self.velocity_limits_raw[mid] = value
            return f"OK: set_goal_velocity {parts[1]} {value}"
        if head == "set_total_current_lim":
            self.total_current_budget_mA = max(1, int(float(parts[1])))
            return f"OK: total_current_limit {self.total_current_budget_mA}"
        if head == "set_control_mode":
            self.control_mode = parts[2]
            return (
                f"Motor control mode: {self.control_mode} "
                "(torque remains off until explicitly enabled)"
            )
        if head == "set_angle":
            mid, value = int(parts[1]), float(parts[2])
            low, high = self.limits[mid]
            self.angles[mid] = min(high, max(low, value))
            return f"OK: angle id={mid} angle={self.angles[mid]:.3f}"
        if head == "hold_position":
            mid, value = int(parts[1]), float(parts[2])
            requested = int(float(parts[3])) if len(parts) > 3 else 25
            applied = min(
                requested,
                self.current_limits[mid],
                910,
                self.total_current_budget_mA,
            )
            self.enabled[mid] = True
            self.holds[mid] = {"angle": value, "current_mA": applied}
            self.angles[mid] = value
            return (
                f"OK: hold_position id={mid} angle={value:.3f} "
                f"current_mA={applied}"
            )
        if head == "release_hold":
            mid = int(parts[1])
            self.holds.pop(mid, None)
            self.enabled[mid] = False
            return f"OK: release_hold id={mid}"
        if head == "set_command_timeout":
            return f"Direct command timeout: {parts[1]} ms"
        if head in {"stop", "set_velocity", "set_current"}:
            return f"OK: {head}"
        return f"ERROR: unsupported command {head}"
