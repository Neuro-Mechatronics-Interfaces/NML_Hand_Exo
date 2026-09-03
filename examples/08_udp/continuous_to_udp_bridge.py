"""Versioned continuous-vector UDP output for the N-joint decoder.

Wire contract
-------------
Each datagram is one UTF-8 JSON object with no trailing newline::

    {"schema":"nml.continuous.v1","sequence":17,"source_time_s":1234.56,
     "channel_names":["Thumb","Index","Pinky"],
     "values":[0.25,-0.5,0.0]}

``channel_names[i]`` identifies ``values[i]``. Values are finite and clipped to
[-1, 1]. A matching binary ``NGA3`` acknowledgement permits the next frame to
be sent immediately; otherwise ``send_every_n_steps`` bounds how long the
sender waits before transmitting the newest sample. See
``CONTINUOUS_UDP_SCHEMA.md`` beside this module for the complete contract.
"""

from __future__ import annotations

import json
import math
import queue
import socket
import struct
import threading
import time
from typing import Any, Mapping, Sequence

import numpy as np

from ctrl import ctrlr
from ctrl.core import Transformer
from ctrl.logging import get_logger
from pybmi.utils.event_api_formatting import format_for_event_api


logger = get_logger(__name__)

SCHEMA = "nml.continuous.v1"
SEQUENCE_MODULUS = 2**32
ACK_MAGIC = b"NGA3"
ACK_HEADER = struct.Struct("<4sIB")
ACK_JOINT_ORDER = ("thumb", "index", "middle", "ring", "pinky", "wrist")


def _validate_channel_names(names: Sequence[str], width: int) -> list[str]:
    result = [str(name).strip() for name in names]
    if len(result) != int(width):
        raise ValueError(
            f"continuous UDP channel-name count {len(result)} != value width {width}"
        )
    if any(not name for name in result):
        raise ValueError("continuous UDP channel names must be non-empty")
    if len({name.casefold() for name in result}) != len(result):
        raise ValueError("continuous UDP channel names must be unique")
    return result


def encode_continuous_packet(
    values: Sequence[float] | np.ndarray,
    channel_names: Sequence[str],
    *,
    sequence: int,
    source_time_s: float,
) -> bytes:
    """Encode one strict ``nml.continuous.v1`` UDP datagram."""
    vector = np.asarray(values, dtype=np.float32).reshape(-1)
    names = _validate_channel_names(channel_names, len(vector))
    if not len(vector):
        raise ValueError("continuous UDP vectors must contain at least one channel")
    if not np.all(np.isfinite(vector)):
        raise ValueError("continuous UDP values must be finite")
    if isinstance(sequence, bool) or not isinstance(sequence, (int, np.integer)):
        raise ValueError("continuous UDP sequence must be an integer")
    if isinstance(source_time_s, bool):
        raise ValueError("continuous UDP source_time_s must be numeric")
    timestamp = float(source_time_s)
    if not math.isfinite(timestamp):
        raise ValueError("continuous UDP source_time_s must be finite")
    payload = {
        "schema": SCHEMA,
        "sequence": int(sequence) % SEQUENCE_MODULUS,
        "source_time_s": timestamp,
        "channel_names": names,
        "values": np.clip(vector, -1.0, 1.0).astype(float).tolist(),
    }
    return json.dumps(
        payload,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def decode_continuous_packet(data: bytes) -> dict[str, Any]:
    """Strict reference decoder suitable for reuse by the Exo handler."""
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("continuous UDP datagram is not valid UTF-8 JSON") from exc
    if not isinstance(payload, Mapping) or payload.get("schema") != SCHEMA:
        raise ValueError(f"continuous UDP schema must equal {SCHEMA!r}")
    required = {"schema", "sequence", "source_time_s", "channel_names", "values"}
    if set(payload) != required:
        raise ValueError(
            "continuous UDP fields must be exactly " + ", ".join(sorted(required))
        )
    sequence = payload["sequence"]
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise ValueError("continuous UDP sequence must be an integer")
    if not 0 <= sequence < SEQUENCE_MODULUS:
        raise ValueError("continuous UDP sequence must be uint32")
    if isinstance(payload["source_time_s"], bool):
        raise ValueError("continuous UDP source_time_s must be numeric")
    timestamp = float(payload["source_time_s"])
    if not math.isfinite(timestamp):
        raise ValueError("continuous UDP source_time_s must be finite")
    raw_values = payload["values"]
    raw_names = payload["channel_names"]
    if not isinstance(raw_values, list) or not isinstance(raw_names, list):
        raise ValueError("continuous UDP channel_names and values must be arrays")
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in raw_values
    ):
        raise ValueError("continuous UDP values must contain only JSON numbers")
    values = np.asarray(raw_values, dtype=np.float32)
    if values.ndim != 1 or not len(values) or not np.all(np.isfinite(values)):
        raise ValueError("continuous UDP values must be a non-empty finite vector")
    if np.any(values < -1.0) or np.any(values > 1.0):
        raise ValueError("continuous UDP values must lie in [-1, 1]")
    names = _validate_channel_names(raw_names, len(values))
    return {
        "schema": SCHEMA,
        "sequence": int(sequence),
        "source_time_s": timestamp,
        "channel_names": names,
        "values": values.tolist(),
    }


def decode_continuous_ack(data: bytes) -> dict[str, Any]:
    """Decode one strict little-endian ``NGA3`` acknowledgement frame."""
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise ValueError("continuous UDP ack must be bytes")
    frame = bytes(data)
    if len(frame) < ACK_HEADER.size:
        raise ValueError("continuous UDP ack is shorter than its 9-byte header")
    magic, sequence, count = ACK_HEADER.unpack_from(frame)
    if magic != ACK_MAGIC:
        raise ValueError("continuous UDP ack magic must equal NGA3")
    if not 1 <= int(count) <= len(ACK_JOINT_ORDER):
        raise ValueError(
            f"continuous UDP ack count must lie in [1, {len(ACK_JOINT_ORDER)}]"
        )
    expected = ACK_HEADER.size + int(count)
    if len(frame) != expected:
        raise ValueError(
            f"continuous UDP ack length {len(frame)} != expected {expected}"
        )
    values = list(struct.unpack_from(f"<{int(count)}b", frame, ACK_HEADER.size))
    if any(value < -100 or value > 100 for value in values):
        raise ValueError("continuous UDP ack values must lie in [-100, 100]")
    joint_names = list(ACK_JOINT_ORDER[: int(count)])
    return {
        "magic": ACK_MAGIC.decode("ascii"),
        "sequence": int(sequence),
        "count": int(count),
        "joint_names": joint_names,
        "values": values,
        "joint_values": dict(zip(joint_names, values)),
    }


class ContinuousToUDPSender(Transformer):
    """Send on ACK, with a bounded step-based fallback when an ACK is late."""

    def __init__(
        self,
        target_ip: str = "172.24.44.148",
        target_port: int = 10003,
        localhost_ip: str | None = None,
        localhost_port: int | None = None,
        listen_ip: str = "0.0.0.0",
        listen_port: int | None = None,
        send_every_n_steps: int = 3,
        input_min: float = -1.0,
        input_max: float = 1.0,
        channel_names: Sequence[str] | None = None,
        allow: bool = True,
        rearm_send: bool = False,
    ) -> None:
        super().__init__(
            name="model.continuous_to_udp_sender",
            in_port_names=["main"],
            out_port_names=["ack"],
        )
        if int(send_every_n_steps) < 1:
            raise ValueError("send_every_n_steps must be positive")
        if not math.isfinite(float(input_min)) or not math.isfinite(float(input_max)):
            raise ValueError("continuous UDP input range must be finite")
        if float(input_max) <= float(input_min):
            raise ValueError("continuous UDP input_max must exceed input_min")

        self._addr = (str(target_ip), int(target_port))
        self._localhost_addr = None
        if localhost_port is not None:
            self._localhost_addr = (
                str(localhost_ip or "127.0.0.1"),
                int(localhost_port),
            )
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._ack_queue: queue.SimpleQueue[dict[str, Any]] = queue.SimpleQueue()
        self._listener_stop = threading.Event()
        self._listener_thread: threading.Thread | None = None
        self._listening = False
        self._send_credit = True
        self._outstanding_sequence: int | None = None
        self._outstanding_sent_at: float | None = None
        self._outstanding_sent_step: int | None = None
        self._pending_sample: tuple[np.ndarray, list[str], float] | None = None
        self._ack_warned = False
        try:
            self._sock.bind((str(listen_ip), int(listen_port or 0)))
            self._sock.settimeout(0.1)
            self._listening = True
        except Exception as exc:
            logger.warning(
                f"Continuous UDP ack bind to {listen_ip}:{listen_port or 0} "
                f"failed ({exc}); using the step-based send fallback"
            )
        self.register_param(
            "send_every_n_steps", int(send_every_n_steps), type="integer"
        )
        self.register_param("input_min", float(input_min), type="float")
        self.register_param("input_max", float(input_max), type="float")
        self.register_param("allow", bool(allow), type="boolean")
        self.register_param(
            "rearm_send",
            bool(rearm_send),
            type="boolean",
            callback=self._on_rearm_send,
        )
        self._configured_channel_names = (
            [str(name) for name in channel_names] if channel_names is not None else None
        )
        self._resolved_channel_names: list[str] | None = None
        self._step = 0
        self._sequence = 0
        self._send_warned = False

        logger.warning(
            f"Continuous UDP sender -> {self._addr[0]}:{self._addr[1]} "
            f"(send on ACK, or after at most "
            f"{int(self.send_every_n_steps)} input steps)"
        )
        if self._listening:
            bound = self._sock.getsockname()
            logger.warning(
                f"Continuous UDP NGA3 ack listener bound to {bound[0]}:{bound[1]}"
            )

    @classmethod
    def version(cls) -> str:
        return "20260902.2"  # nml.continuous.v2 ACK pacing with step fallback

    def setup(self) -> None:
        self.out_ports["ack"].metadata = {
            "signal_type": "continuous_udp_ack",
            "event_type": "continuous_udp_ack",
            "wire_magic": ACK_MAGIC.decode("ascii"),
            "joint_order": json.dumps(list(ACK_JOINT_ORDER)),
        }
        if self._listening and self._listener_thread is None:
            self._listener_stop.clear()
            self._listener_thread = threading.Thread(
                target=self._ack_listener_loop,
                name="continuous_udp_ack_listener",
                daemon=True,
            )
            self._listener_thread.start()

    def _on_rearm_send(self) -> None:
        """Queue a momentary manual send credit from the live boolean param."""
        if not bool(getattr(self, "rearm_send", False)):
            return
        self._ack_queue.put(
            {
                "kind": "manual_rearm",
                "received_time_s": ctrlr.time_s(),
            }
        )
        # register_param attributes can be cleared without re-firing callbacks.
        self.rearm_send = False

    def _ack_listener_loop(self) -> None:
        """Receive/parse ACKs off-thread and communicate only through a queue."""
        while not self._listener_stop.is_set():
            try:
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                if not self._listener_stop.is_set():
                    logger.warning(f"Continuous UDP ack receive failed: {exc}")
                continue
            self._queue_ack(data, addr)

    def _queue_ack(self, data: bytes, addr: tuple[str, int]) -> None:
        """Parse on the listener side; never touch ports or send credit here."""
        try:
            ack = decode_continuous_ack(data)
        except ValueError as exc:
            if not self._ack_warned:
                logger.warning(f"Ignoring malformed continuous UDP ack: {exc}")
                self._ack_warned = True
            return
        self._ack_warned = False
        self._ack_queue.put(
            {
                "kind": "ack",
                "ack": ack,
                "addr": (str(addr[0]), int(addr[1])),
                "received_time_s": ctrlr.time_s(),
                "received_monotonic": time.monotonic(),
            }
        )

    def _drain_ack_queue(self) -> None:
        """Apply credits and emit normalized ACK events on the pipeline thread."""
        events: list[dict[str, Any]] = []
        timestamps: list[float] = []
        while True:
            try:
                entry = self._ack_queue.get_nowait()
            except queue.Empty:
                break
            if entry.get("kind") == "manual_rearm":
                abandoned = self._outstanding_sequence
                self._send_credit = True
                self._outstanding_sequence = None
                self._outstanding_sent_at = None
                self._outstanding_sent_step = None
                logger.warning(
                    "Continuous UDP send credit manually restored"
                    + (f"; abandoned sequence {abandoned}" if abandoned is not None else "")
                )
                continue

            ack = entry["ack"]
            sequence = int(ack["sequence"])
            matched = sequence == self._outstanding_sequence
            sent_at = self._outstanding_sent_at if matched else None
            if matched:
                self._send_credit = True
                self._outstanding_sequence = None
                self._outstanding_sent_at = None
                self._outstanding_sent_step = None
            rtt_ms = (
                (float(entry["received_monotonic"]) - float(sent_at)) * 1000.0
                if sent_at is not None
                else None
            )
            addr = entry["addr"]
            payload = {
                **ack,
                "peer": f"{addr[0]}:{addr[1]}",
                "matched_outstanding": bool(matched),
                "credit_restored": bool(matched),
                "round_trip_ms": rtt_ms,
            }
            events.append(
                format_for_event_api(
                    event_name="ContinuousAck",
                    timing="instant",
                    event_type="continuous_udp_ack",
                    payload=payload,
                )
            )
            timestamps.append(float(entry["received_time_s"]))
        if events:
            try:
                self.out_ports["ack"].send(events, timestamps)
            except Exception as exc:
                logger.warning(f"Continuous UDP ack output failed: {exc}")

    @staticmethod
    def _metadata_channel_names(metadata: Mapping[str, Any]) -> list[str] | None:
        raw = metadata.get("output_layout", metadata.get("ch_names"))
        if raw is None:
            return None
        if isinstance(raw, str):
            try:
                decoded = json.loads(raw)
                raw = decoded if isinstance(decoded, list) else raw.split(",")
            except json.JSONDecodeError:
                raw = raw.split(",")
        if not isinstance(raw, Sequence) or isinstance(raw, (bytes, bytearray)):
            return None
        return [str(name).strip() for name in raw]

    def _channel_names(self, width: int) -> list[str]:
        candidate = self._configured_channel_names
        if candidate is None:
            candidate = self._metadata_channel_names(self.in_ports["main"].metadata)
        if candidate is None:
            raise ValueError(
                "continuous UDP input metadata has no output_layout; configure channel_names"
            )
        candidate = _validate_channel_names(candidate, width)
        if self._resolved_channel_names is None:
            self._resolved_channel_names = candidate
        elif candidate != self._resolved_channel_names:
            raise ValueError("continuous UDP channel layout changed while streaming")
        return self._resolved_channel_names

    def _scale(self, row: np.ndarray) -> np.ndarray:
        lo = float(self.input_min)
        hi = float(self.input_max)
        if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
            raise ValueError("continuous UDP input range became invalid")
        return np.clip(2.0 * (row - lo) / (hi - lo) - 1.0, -1.0, 1.0)

    def _send(self, packet: bytes) -> None:
        # The target send is authoritative: once it succeeds the frame may have
        # reached the exo and its credit must be consumed even if a debug mirror
        # fails afterward.
        self._sock.sendto(packet, self._addr)
        if self._localhost_addr is not None:
            try:
                self._sock.sendto(packet, self._localhost_addr)
            except OSError as exc:
                logger.warning(f"Continuous UDP localhost mirror failed: {exc}")

    def _send_pending_if_permitted(self, *, max_wait_elapsed: bool = False) -> None:
        if (
            self._pending_sample is None
            or not bool(self.allow)
            or (not self._send_credit and not max_wait_elapsed)
        ):
            return
        row, names, timestamp = self._pending_sample
        packet = encode_continuous_packet(
            self._scale(row),
            names,
            sequence=self._sequence,
            source_time_s=timestamp,
        )
        try:
            self._send(packet)
        except OSError as exc:
            if not self._send_warned:
                logger.warning(f"Continuous UDP send failed: {exc}")
                self._send_warned = True
            return
        self._send_warned = False
        sent_sequence = self._sequence
        self._sequence = (self._sequence + 1) % SEQUENCE_MODULUS
        self._send_credit = False
        self._outstanding_sequence = sent_sequence
        self._outstanding_sent_at = time.monotonic()
        self._outstanding_sent_step = self._step
        self._pending_sample = None

    def stream_transform(self) -> None:
        # The listener and parameter callback communicate only through this
        # queue. Port writes and flow-control changes therefore remain on the
        # pipeline thread.
        self._drain_ack_queue()
        # If an ACK arrived while a sample was waiting, release that newest held
        # sample immediately on this pipeline cycle.
        self._send_pending_if_permitted()

        batch, times = self.in_ports["main"].get_all()
        if not batch:
            return
        now = ctrlr.time_s()
        if len(times) != len(batch):
            times = [now] * len(batch)
        every = int(self.send_every_n_steps)
        if every < 1:
            raise ValueError("send_every_n_steps must remain positive")

        for sample, timestamp in zip(batch, times):
            self._step += 1
            row = np.asarray(sample, dtype=np.float32).reshape(-1)
            if not len(row) or not np.all(np.isfinite(row)):
                logger.warning("Dropping non-finite or empty continuous UDP row")
                continue
            names = self._channel_names(len(row))
            # While waiting for an ACK, retain the newest decoder row. The ACK
            # releases it early; the configured step count is the maximum wait,
            # so a lost ACK can never gate the sender indefinitely.
            self._pending_sample = (row.copy(), names, float(timestamp))
            max_wait_elapsed = (
                not self._send_credit
                and self._outstanding_sent_step is not None
                and self._step - self._outstanding_sent_step >= every
            )
            self._send_pending_if_permitted(max_wait_elapsed=max_wait_elapsed)

    def teardown(self) -> None:
        self._listener_stop.set()
        if self._listener_thread is not None:
            self._listener_thread.join(timeout=0.5)
            self._listener_thread = None
        try:
            self._sock.close()
        except OSError as exc:
            logger.warning(f"Continuous UDP socket close failed: {exc}")
        super().teardown()


__all__ = [
    "SCHEMA",
    "SEQUENCE_MODULUS",
    "ACK_MAGIC",
    "ACK_JOINT_ORDER",
    "ContinuousToUDPSender",
    "decode_continuous_ack",
    "decode_continuous_packet",
    "encode_continuous_packet",
]
