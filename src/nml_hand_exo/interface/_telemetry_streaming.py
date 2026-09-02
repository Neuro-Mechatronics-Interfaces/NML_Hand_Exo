from __future__ import annotations

import json
import socket
import time
from typing import Iterable, Mapping, Sequence


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


class StructuredLSLTelemetryOutlet:
    """Versioned numeric LSL outlet with per-channel metadata and units.

    ``NumericLSLTelemetryOutlet`` remains unchanged for the legacy angle and
    torque streams.  This outlet supports heterogeneous quantities in one
    fixed-order state frame while keeping channel order explicit in metadata.
    """

    def __init__(
        self,
        name: str,
        stream_type: str,
        source_id: str,
        schema: str,
    ) -> None:
        self.name = str(name)
        self.stream_type = str(stream_type)
        self.source_id = str(source_id)
        self.schema = str(schema)
        self.enabled = False
        self._channel_specs: list[dict[str, str]] = []
        self._stream_metadata: dict[str, str] = {}
        self._nominal_srate = 2.0
        self._outlet = None
        self._local_clock = None
        self._last_error: str | None = None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    @property
    def channel_labels(self) -> tuple[str, ...]:
        return tuple(spec["label"] for spec in self._channel_specs)

    def configure(
        self,
        enabled: bool,
        channel_specs: Sequence[Mapping[str, object]],
        nominal_srate: float = 2.0,
        stream_metadata: Mapping[str, object] | None = None,
    ) -> None:
        specs = []
        for index, source in enumerate(channel_specs):
            spec = {str(key): str(value) for key, value in source.items() if value is not None}
            if not spec.get("label", "").strip():
                raise ValueError(f"LSL channel {index} has no label")
            specs.append(spec)
        nominal_srate = float(nominal_srate)
        metadata = {
            str(key): str(value)
            for key, value in (stream_metadata or {}).items()
            if value is not None
        }
        if not enabled:
            self.close()
            self.enabled = False
            self._channel_specs = specs
            self._nominal_srate = nominal_srate
            self._stream_metadata = metadata
            return
        if (
            self.enabled
            and self._channel_specs == specs
            and self._nominal_srate == nominal_srate
            and self._stream_metadata == metadata
            and self._outlet is not None
        ):
            return
        self.close()
        self.enabled = True
        self._channel_specs = specs
        self._nominal_srate = nominal_srate
        self._stream_metadata = metadata
        self._last_error = None
        if not specs:
            return
        try:
            from pylsl import StreamInfo, StreamOutlet, cf_float32, local_clock

            info = StreamInfo(
                name=self.name,
                type=self.stream_type,
                channel_count=len(specs),
                nominal_srate=nominal_srate,
                channel_format=cf_float32,
                source_id=self.source_id,
            )
            info.desc().append_child_value("schema", self.schema)
            for key, value in metadata.items():
                info.desc().append_child_value(key, value)
            channels = info.desc().append_child("channels")
            for spec in specs:
                channel = channels.append_child("channel")
                for key, value in spec.items():
                    channel.append_child_value(key, value)
            self._outlet = StreamOutlet(info)
            self._local_clock = local_clock
        except Exception as exc:
            self.enabled = False
            self._last_error = str(exc)
            self._outlet = None
            self._local_clock = None

    def publish(
        self,
        values_by_channel: Mapping[str, float | int | bool | None],
        timestamp: float | None = None,
    ) -> None:
        if not self.enabled or self._outlet is None:
            return
        sample = []
        for spec in self._channel_specs:
            value = values_by_channel.get(spec["label"])
            sample.append(float("nan") if value is None else float(value))
        ts = (
            float(timestamp)
            if timestamp is not None
            else (self._local_clock() if self._local_clock else time.time())
        )
        try:
            self._outlet.push_sample(sample, ts)
        except Exception as exc:
            self._last_error = str(exc)

    def close(self) -> None:
        self._outlet = None
        self._local_clock = None


class StringLSLEventOutlet:
    """Irregular JSON event stream with a versioned LSL schema."""

    def __init__(
        self,
        name: str,
        stream_type: str,
        source_id: str,
        schema: str,
    ) -> None:
        self.name = str(name)
        self.stream_type = str(stream_type)
        self.source_id = str(source_id)
        self.schema = str(schema)
        self.enabled = False
        self._outlet = None
        self._local_clock = None
        self._last_error: str | None = None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def configure(self, enabled: bool) -> None:
        if not enabled:
            self.close()
            self.enabled = False
            return
        if self.enabled and self._outlet is not None:
            return
        self.close()
        self.enabled = True
        self._last_error = None
        try:
            from pylsl import StreamInfo, StreamOutlet, cf_string, local_clock

            info = StreamInfo(
                name=self.name,
                type=self.stream_type,
                channel_count=1,
                nominal_srate=0.0,
                channel_format=cf_string,
                source_id=self.source_id,
            )
            info.desc().append_child_value("schema", self.schema)
            channels = info.desc().append_child("channels")
            channel = channels.append_child("channel")
            channel.append_child_value("label", "event_json")
            channel.append_child_value("unit", "json")
            self._outlet = StreamOutlet(info)
            self._local_clock = local_clock
        except Exception as exc:
            self.enabled = False
            self._last_error = str(exc)
            self._outlet = None
            self._local_clock = None

    def publish(self, event: Mapping[str, object], timestamp: float | None = None) -> None:
        if not self.enabled or self._outlet is None:
            return
        try:
            payload = json.dumps(
                dict(event), separators=(",", ":"), sort_keys=True, allow_nan=False
            )
            ts = (
                float(timestamp)
                if timestamp is not None
                else (self._local_clock() if self._local_clock else time.time())
            )
            self._outlet.push_sample([payload], ts)
        except Exception as exc:
            self._last_error = str(exc)

    def close(self) -> None:
        self._outlet = None
        self._local_clock = None
