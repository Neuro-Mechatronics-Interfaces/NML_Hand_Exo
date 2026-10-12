import struct
import unittest

from nml_hand_exo.interface import HandExo


class FakeRawSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.is_open = True

    def feed(self, payload: bytes):
        self.buffer.extend(payload)

    def reset_input_buffer(self):
        self.buffer.clear()

    def read(self, count=1):
        payload = self.buffer[:count]
        del self.buffer[:count]
        return bytes(payload)


class FakeFastTelemetryComm:
    def __init__(self, frame: bytes):
        self.verbose = False
        self.raw = FakeRawSerial()
        self.frame = frame
        self.sent = []

    def fast_telemetry_device(self):
        return self.raw

    def send(self, message: str):
        self.sent.append(message)
        if message.startswith("get_telemetry_fast:"):
            self.raw.feed(self.frame)


def make_frame():
    header_fmt = "<2sBBBHIH"
    record = struct.pack("<BBhiiii", 11, 0, 123, -4, 2048, 18025, -975)
    header_without_checksum = struct.pack(
        header_fmt, b"NX", 1, 2, 1, len(record), 4567, 0
    )
    checksum = (sum(header_without_checksum[:-2]) + sum(record)) & 0xFFFF
    header = struct.pack(
        header_fmt, b"NX", 1, 2, 1, len(record), 4567, checksum
    )
    return header + record


def make_control_frame():
    record_fmt = "<BBhiiiiBQIiiiihhHHHBB"
    record = struct.pack(
        record_fmt,
        11, 0, 123, -4, 2048, 18025, -975,
        0x15, 0, 4568,
        18250, 18100, 18250, 225,
        300, 42, 500, 0, 125, 6, 0x1F,
    )
    header_fmt = "<2sBBBHIH"
    header_without_checksum = struct.pack(
        header_fmt, b"NX", 3, 2, 1, len(record), 4567, 0
    )
    checksum = (sum(header_without_checksum[:-2]) + sum(record)) & 0xFFFF
    header = struct.pack(
        header_fmt, b"NX", 3, 2, 1, len(record), 4567, checksum
    )
    return header + record


class FastTelemetryTests(unittest.TestCase):
    def test_reads_frame_through_transport_raw_stream(self):
        comm = FakeFastTelemetryComm(make_frame())
        exo = HandExo(comm, command_delimiter="\r\n", send_delay=0)

        result = exo.get_fast_telemetry(timeout=0.1, motor_ids=[11])

        self.assertEqual(comm.sent, ["get_telemetry_fast:11\r\n"])
        self.assertEqual(result[11]["current"], 123)
        self.assertEqual(result[11]["absolute_angle"], 180.25)
        self.assertEqual(result[11]["angle"], -9.75)
        self.assertEqual(result[11]["flags"], 2)

    def test_reads_v3_control_loop_observability(self):
        comm = FakeFastTelemetryComm(make_control_frame())
        exo = HandExo(comm, command_delimiter="\r\n", send_delay=0)

        result = exo.get_fast_telemetry(timeout=0.1, motor_ids=[11])

        self.assertEqual(result[11]["frame_version"], 3)
        self.assertEqual(result[11]["goal_angle"], 182.5)
        self.assertEqual(result[11]["secondary_goal_angle"], 181.0)
        self.assertEqual(result[11]["reference_angle"], 182.5)
        self.assertEqual(result[11]["position_error"], 2.25)
        self.assertEqual(result[11]["goal_current"], 300)
        self.assertEqual(result[11]["assist_effort"], 42)
        self.assertEqual(result[11]["intent_ms"], 500)
        self.assertEqual(result[11]["stall_ms"], 125)
        self.assertEqual(result[11]["control_state"], 6)
        self.assertEqual(result[11]["control_flags"], 0x1F)


if __name__ == "__main__":
    unittest.main()
