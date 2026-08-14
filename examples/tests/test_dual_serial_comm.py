"""Tests for the decoupled DualSerialComm transport.

These exercise the reader thread and frame splitting against a fake serial
port, so no hardware is involved.  The property under test is decoupling: the
telemetry port is drained continuously, and a reply that never arrives must not
stall the caller beyond its own timeout or leave a backlog behind.
"""

import threading
import time
import unittest
from unittest import mock

from nml_hand_exo import DualSerialComm


class FakeSerial:
    """Minimal pyserial stand-in supporting the reader thread's usage."""

    def __init__(self, timeout=0.05):
        self.is_open = True
        self.timeout = timeout
        self.written = bytearray()
        self._buf = bytearray()
        self._lock = threading.Lock()

    def feed(self, text):
        """Push device->host bytes onto the port."""
        with self._lock:
            self._buf.extend(text.encode() if isinstance(text, str) else text)

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._buf)

    def read(self, n=1):
        deadline = time.monotonic() + (self.timeout or 0)
        while True:
            with self._lock:
                if self._buf:
                    take = self._buf[:n]
                    del self._buf[:n]
                    return bytes(take)
            if time.monotonic() >= deadline:
                return b""
            time.sleep(0.001)

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def reset_input_buffer(self):
        with self._lock:
            self._buf.clear()

    def close(self):
        self.is_open = False


class DualSerialCommReaderTests(unittest.TestCase):
    def _make(self):
        comm = DualSerialComm(
            cmd_port="FAKE_CMD", telem_port="FAKE_TELEM", baudrate=1000000,
            response_timeout=0.5, timeout=0.05,
        )
        comm._cmd = FakeSerial()
        comm._telem = FakeSerial()
        comm._start_reader()
        self.addCleanup(comm.close)
        return comm

    def test_single_line_reply_is_framed(self):
        comm = self._make()
        comm._telem.feed("Exo Device Version: 1.0;\r\n")
        self.assertEqual(comm.receive(wait_until_return=True, timeout=1.0),
                         "Exo Device Version: 1.0")

    def test_multi_line_reply_is_one_frame(self):
        comm = self._make()
        comm._telem.feed(
            "Motor absolute angles:\r\n"
            "Motor 0: {name: index, id: 1}\r\n"
            "Motor 1: {name: middle, id: 2};\r\n"
        )
        frame = comm.receive(wait_until_return=True, timeout=1.0)
        self.assertIn("Motor 0: {name: index, id: 1}", frame)
        self.assertIn("Motor 1: {name: middle, id: 2}", frame)
        self.assertNotIn(";", frame)

    def test_unsolicited_output_does_not_produce_a_reply(self):
        comm = self._make()
        comm._telem.feed("[GestureController] targets: 1->10.0\r\n"
                         "Received: set_gesture:grasp:close\r\n")
        # Nothing delimited arrived, so a non-blocking read yields nothing...
        time.sleep(0.15)
        self.assertEqual(comm.receive(), "")
        # ...but the lines were still captured for observability.
        self.assertEqual(len(comm.telemetry_lines()), 2)

    def test_reply_after_debug_noise_still_resolves(self):
        comm = self._make()
        comm._telem.feed("Received: version\r\n"
                         "[GestureController] noise\r\n"
                         "OK: gesture grasp:close;\r\n")
        frame = comm.receive(wait_until_return=True, timeout=1.0)
        self.assertIn("OK: gesture grasp:close", frame)

    def test_timeout_returns_empty_and_does_not_block_the_writer(self):
        comm = self._make()
        start = time.monotonic()
        self.assertEqual(comm.receive(wait_until_return=True, timeout=0.2), "")
        elapsed = time.monotonic() - start
        self.assertLess(elapsed, 1.0, "receive overran its timeout")
        # The command port is still immediately writable after a missed reply.
        comm.send("version\r\n")
        self.assertIn(b"version", bytes(comm._cmd.written))

    def test_expected_timeout_can_be_silent(self):
        comm = self._make()
        with mock.patch("builtins.print") as printed:
            self.assertEqual(
                comm.receive(
                    wait_until_return=True,
                    timeout=0.05,
                    warn_on_timeout=False,
                ),
                "",
            )
        printed.assert_not_called()

    def test_queued_reply_returns_without_waiting(self):
        comm = self._make()
        comm._telem.feed("OK: gesture grasp:open;\r\n")
        time.sleep(0.15)  # let the reader pick it up
        start = time.monotonic()
        frame = comm.receive(wait_until_return=True, timeout=5.0)
        self.assertIn("OK: gesture grasp:open", frame)
        self.assertLess(time.monotonic() - start, 0.1,
                        "a queued reply should return immediately")

    def test_flush_input_drops_stale_replies(self):
        comm = self._make()
        comm._telem.feed("STALE;\r\n")
        time.sleep(0.15)
        comm.flush_input()
        self.assertEqual(comm.receive(), "")

    def test_backlog_does_not_accumulate_across_transactions(self):
        comm = self._make()
        for i in range(20):
            comm._telem.feed(f"noise {i}\r\n")
        comm._telem.feed("OK: final;\r\n")
        frame = comm.receive(wait_until_return=True, timeout=1.0)
        self.assertIn("OK: final", frame)
        # Port fully drained: no second frame is hiding behind the first.
        self.assertEqual(comm.receive(), "")

    def test_dead_reader_is_reported_not_silently_timed_out(self):
        """A failed reader must be distinguishable from a quiet device."""
        class Exploding(FakeSerial):
            def read(self, n=1):
                raise OSError("simulated port failure")

        comm = DualSerialComm(
            cmd_port="FAKE_CMD", telem_port="FAKE_TELEM", baudrate=1000000,
            response_timeout=0.2, timeout=0.05,
        )
        comm._cmd = FakeSerial()
        comm._telem = Exploding()
        comm._start_reader()
        self.addCleanup(comm.close)

        for _ in range(50):
            if not comm.reader_alive():
                break
            time.sleep(0.01)

        self.assertFalse(comm.reader_alive())
        self.assertIsNotNone(comm._reader_error)
        self.assertEqual(comm.receive(wait_until_return=True, timeout=0.1), "")

    def test_close_does_not_record_a_reader_error(self):
        comm = self._make()
        comm.close()
        self.assertIsNone(comm._reader_error)

    def test_close_stops_the_reader_thread(self):
        comm = self._make()
        reader = comm._reader
        self.assertTrue(reader.is_alive())
        comm.close()
        self.assertFalse(reader.is_alive())

    def test_fast_telemetry_uses_unclaimed_command_stream(self):
        comm = self._make()
        self.assertIs(comm.fast_telemetry_device(), comm._cmd)


if __name__ == "__main__":
    unittest.main()
