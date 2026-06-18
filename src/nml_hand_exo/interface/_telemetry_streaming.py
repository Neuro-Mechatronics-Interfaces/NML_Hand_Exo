from __future__ import annotations

import json
import socket
import time
from typing import Iterable


class UDPTelemetryPublisher:
    """Publish telemetry frames as compact JSON over UDP."""

    def __init__(self) -> None:
        self.enabled = False
        self.host = "127.0.0.1"
        self.port = 10002
        self._sock: socket.socket | None = None

    def configure(self, enabled: bool, host: str, port: int) -> None:
        self.enabled = bool(enabled)
        self.host = host.strip() or "127.0.0.1"
        self.port = int(port)
        if self.enabled and self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not self.enabled:
            self.close()

    def publish(self, frame: dict) -> None:
        if not self.enabled:
            return
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = json.dumps(frame, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._sock.sendto(payload, (self.host, self.port))

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class NumericLSLTelemetryOutlet:
    """Small wrapper around a continuous numeric LSL outlet."""

    def __init__(self, name: str, stream_type: str, unit: str) -> None:
        self.name = name
        self.stream_type = stream_type
        self.unit = unit
        self.enabled = False
        self._channel_names: list[str] = []
        self._nominal_srate = 2.0
        self._outlet = None
        self._local_clock = None
        self._last_error: str | None = None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def configure(
        self,
        enabled: bool,
        channel_names: Iterable[str],
        nominal_srate: float = 2.0,
    ) -> None:
        names = list(channel_names)
        nominal_srate = float(nominal_srate)
        if not enabled:
            self.close()
            self.enabled = False
            self._channel_names = names
            self._nominal_srate = nominal_srate
            return
        if (
            self.enabled
            and self._channel_names == names
            and self._nominal_srate == nominal_srate
            and self._outlet is not None
        ):
            return
        self.close()
        self.enabled = True
        self._channel_names = names
        self._nominal_srate = nominal_srate
        self._last_error = None
        if not names:
            return
        try:
            from pylsl import StreamInfo, StreamOutlet, cf_float32, local_clock

            info = StreamInfo(
                name=self.name,
                type=self.stream_type,
                channel_count=len(names),
                nominal_srate=nominal_srate,
                channel_format=cf_float32,
                source_id=f"nml_hand_exo_{self.stream_type.lower()}",
            )
            channels = info.desc().append_child("channels")
            for label in names:
                ch = channels.append_child("channel")
                ch.append_child_value("label", label)
                ch.append_child_value("unit", self.unit)
            self._outlet = StreamOutlet(info)
            self._local_clock = local_clock
        except Exception as exc:
            self.enabled = False
            self._last_error = str(exc)
            self._outlet = None
            self._local_clock = None

    def publish(self, values_by_channel: dict[str, float | None]) -> None:
        if not self.enabled or self._outlet is None:
            return
        sample = []
        for name in self._channel_names:
            val = values_by_channel.get(name)
            sample.append(float("nan") if val is None else float(val))
        ts = self._local_clock() if self._local_clock else time.time()
        self._outlet.push_sample(sample, ts)

    def close(self) -> None:
        self._outlet = None
        self._local_clock = None
