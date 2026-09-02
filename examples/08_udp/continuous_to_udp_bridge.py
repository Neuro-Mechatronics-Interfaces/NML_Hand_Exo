"""Versioned continuous-vector UDP output for the N-joint decoder.

Wire contract
-------------
Each datagram is one UTF-8 JSON object with no trailing newline::

    {"schema":"nml.continuous.v1","sequence":17,"source_time_s":1234.56,
     "channel_names":["Thumb","Index","Pinky"],
     "values":[0.25,-0.5,0.0]}

``channel_names[i]`` identifies ``values[i]``. Values are finite and clipped to
[-1, 1]. See ``CONTINUOUS_UDP_SCHEMA.md`` beside this module for the complete
receiver contract and a reference dispatch/validation example.

Suggested future change -- ack-gated flow control (not yet implemented)
----------------------------------------------------------------------
This sender currently emits on a fixed stride (every ``send_every_n_steps`` decoder
samples), independent of whether the receiver/exo can keep up. The ``v2`` receiver
now returns one ``NGA3`` ack per accepted frame, emitted only once the exo has
answered it, so this bridge could instead pace itself to the exo's real throughput:
start with one send credit, spend it to send, and restore it on an incoming ack (or
a manual/parameter trigger). When the exo answers faster than 60 ms the next frame
goes out immediately and throughput rises above the fixed ~16.7 Hz ceiling; when it
is slow or a datagram is lost, the sender backs off instead of piling up frames the
receiver would only drop. This requires reading the ``NGA3`` acks on the send socket
(this bridge is currently send-only) plus an ack-timeout watchdog. See the
"Suggested sender change" section of ``CONTINUOUS_UDP_SCHEMA.md``.
"""

from __future__ import annotations

import json
import math
import socket
from typing import Any, Mapping, Sequence

import numpy as np

from ctrl import ctrlr
from ctrl.core import Transformer
from ctrl.logging import get_logger


logger = get_logger(__name__)

SCHEMA = "nml.continuous.v1"
SEQUENCE_MODULUS = 2**32


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


class ContinuousToUDPSender(Transformer):
    """Send every Nth continuous decoder row as a versioned UDP datagram."""

    def __init__(
        self,
        target_ip: str = "10.0.0.10",
        target_port: int = 10003,
        localhost_ip: str | None = None,
        localhost_port: int | None = None,
        send_every_n_steps: int = 3,
        input_min: float = -1.0,
        input_max: float = 1.0,
        channel_names: Sequence[str] | None = None,
        allow: bool = True,
    ) -> None:
        super().__init__(
            name="model.continuous_to_udp_sender",
            in_port_names=["main"],
            out_port_names=[],
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
        self.register_param(
            "send_every_n_steps", int(send_every_n_steps), type="integer"
        )
        self.register_param("input_min", float(input_min), type="float")
        self.register_param("input_max", float(input_max), type="float")
        self.register_param("allow", bool(allow), type="boolean")
        self._configured_channel_names = (
            [str(name) for name in channel_names] if channel_names is not None else None
        )
        self._resolved_channel_names: list[str] | None = None
        self._step = 0
        self._sequence = 0
        self._send_warned = False

        logger.warning(
            f"Continuous UDP sender -> {self._addr[0]}:{self._addr[1]} "
            f"(every {int(self.send_every_n_steps)} input steps)"
        )

    @classmethod
    def version(cls) -> str:
        return "20260902.0"  # nml.continuous.v1 JSON datagrams

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
        self._sock.sendto(packet, self._addr)
        if self._localhost_addr is not None:
            self._sock.sendto(packet, self._localhost_addr)

    def stream_transform(self) -> None:
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
            if self._step % every:
                continue
            row = np.asarray(sample, dtype=np.float32).reshape(-1)
            if not len(row) or not np.all(np.isfinite(row)):
                logger.warning("Dropping non-finite or empty continuous UDP row")
                continue
            names = self._channel_names(len(row))
            packet = encode_continuous_packet(
                self._scale(row),
                names,
                sequence=self._sequence,
                source_time_s=float(timestamp),
            )
            self._sequence = (self._sequence + 1) % SEQUENCE_MODULUS
            if not bool(self.allow):
                continue
            try:
                self._send(packet)
                self._send_warned = False
            except OSError as exc:
                if not self._send_warned:
                    logger.warning(f"Continuous UDP send failed: {exc}")
                    self._send_warned = True

    def teardown(self) -> None:
        try:
            self._sock.close()
        except OSError as exc:
            logger.warning(f"Continuous UDP socket close failed: {exc}")
        super().teardown()


__all__ = [
    "SCHEMA",
    "SEQUENCE_MODULUS",
    "ContinuousToUDPSender",
    "decode_continuous_packet",
    "encode_continuous_packet",
]
