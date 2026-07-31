import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
)

from udp_gesture_receiver import (  # noqa: E402
    COMMAND_MAP,
    COMMAND_PASSTHROUGH_ACK,
    GESTURE_RESULT_PREFIX,
    JOINTS,
    POSE_ACK_MAGIC,
    POSE_QUERY,
    POSE_UNAVAILABLE,
    MockComm,
    Receiver,
    pack_pose_ack,
    parse_passthrough_command,
    unpack_pose_ack,
)
from nml_hand_exo.interface._hand_exo import (  # noqa: E402
    GESTURE_ANGLE_PREFIX,
    parse_gesture_angles,
)
from nml_hand_exo.interface._udp_command_bindings import (  # noqa: E402
    UDP_CONNECTION_PORT_THRESHOLD,
    UDP_HEARTBEAT_REQUEST_VALUE,
)


class PassthroughValidatorTests(unittest.TestCase):
    """The one command form the receiver accepts directly, for manual tuning."""

    def test_accepts_every_known_joint(self):
        for joint in JOINTS:
            self.assertEqual(
                parse_passthrough_command(f"set_gesture_angle:{joint}:42"),
                f"set_gesture_angle:{joint}:42",
            )

    def test_normalizes_case_and_surrounding_whitespace(self):
        self.assertEqual(
            parse_passthrough_command("  set_gesture_angle:INDEX:12.5 \n"),
            "set_gesture_angle:index:12.5",
        )

    def test_accepts_both_range_endpoints(self):
        self.assertEqual(
            parse_passthrough_command("set_gesture_angle:index:0"),
            "set_gesture_angle:index:0",
        )
        self.assertEqual(
            parse_passthrough_command("set_gesture_angle:index:100"),
            "set_gesture_angle:index:100",
        )

    def test_rejects_out_of_range_percent(self):
        for bad in ("101", "-1", "1e9"):
            self.assertIsNone(
                parse_passthrough_command(f"set_gesture_angle:index:{bad}"), bad
            )

    def test_rejects_unknown_joint(self):
        self.assertIsNone(parse_passthrough_command("set_gesture_angle:elbow:50"))

    def test_rejects_malformed_payloads(self):
        for bad in (
            "",
            None,
            "set_gesture_angle:index",
            "set_gesture_angle:index:50:extra",
            "set_gesture_angle:index:abc",
        ):
            self.assertIsNone(parse_passthrough_command(bad), bad)

    def test_is_not_a_general_command_passthrough(self):
        """The receiver binds 0.0.0.0, so nothing else may reach the serial port."""
        for command in (
            "disable:all",
            "enable:all",
            "home:all",
            "set_gesture:grasp:close",
            "set_current_lim:all:910",
            "set_total_current_lim:9999",
            "reboot:all",
            "set_motor_limits:11:0:360",
        ):
            self.assertIsNone(parse_passthrough_command(command), command)


class PassthroughAckTests(unittest.TestCase):
    def test_ack_sentinel_cannot_collide_with_the_integer_protocol(self):
        # Not a command value...
        self.assertNotIn(COMMAND_PASSTHROUGH_ACK, COMMAND_MAP)
        # ...not the heartbeat...
        self.assertNotEqual(COMMAND_PASSTHROUGH_ACK, UDP_HEARTBEAT_REQUEST_VALUE)
        # ...and clear of the command range, which stays under the port threshold.
        self.assertGreater(COMMAND_PASSTHROUGH_ACK, UDP_CONNECTION_PORT_THRESHOLD)
        self.assertGreater(COMMAND_PASSTHROUGH_ACK, max(abs(v) for v in COMMAND_MAP))



class MoveOutcomeTests(unittest.TestCase):
    """Verdicts travel as their own datagram, not on the command ack."""

    def test_outcome_prefix_is_not_parseable_as_an_integer(self):
        from nml_hand_exo.interface._udp_command_bindings import parse_udp_integer
        line = "GESTURE_RESULT: reached=2 stalled=1 short=0 starved=0"
        self.assertIsNone(parse_udp_integer(line))
        self.assertTrue(line.startswith(GESTURE_RESULT_PREFIX))

    def test_outcome_does_not_retire_a_pending_command_ack(self):
        """An unsolicited report must not ack an unrelated command."""
        sent_upstream = []

        class _Comm:
            def __init__(self):
                self.replies = [
                    "GESTURE_RESULT: reached=0 stalled=7 short=0 starved=11",
                    "OK: gesture_angle index:35.0",
                ]

            def receive(self, *a, **k):
                return self.replies.pop(0) if self.replies else ""

        receiver = Receiver(_Comm(), COMMAND_MAP, echo_replies=False, verbose=False)
        receiver.return_addr = ("127.0.0.1", 10004)
        receiver.send_upstream = lambda v: (sent_upstream.append(int(v)), True)[1]
        receiver.send_text_upstream = lambda t: (sent_upstream.append(t), True)[1]
        receiver._pending.append([2, 1])          # one command awaiting its ack

        receiver.drain_replies()

        # The report went up as text, and the ack still belongs to command 2.
        self.assertEqual(len(sent_upstream), 2)
        self.assertTrue(str(sent_upstream[0]).startswith(GESTURE_RESULT_PREFIX))
        self.assertEqual(sent_upstream[1], 2)
        self.assertEqual(receiver.outcomes, 1)
        self.assertEqual(len(receiver._pending), 0)

class _PoseComm:
    """Comm stub that replays a fixed list of reply frames, then nothing."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.sent = []

    def send(self, message):
        self.sent.append(message.strip())

    def receive(self, *a, **k):
        return self.replies.pop(0) if self.replies else ""

    def flush_input(self):
        pass


def _receiver(replies, **kwargs):
    receiver = Receiver(_PoseComm(replies), COMMAND_MAP, echo_replies=False,
                        verbose=False, **kwargs)
    receiver.return_addr = ("127.0.0.1", 10004)
    return receiver


class GestureAngleParsingTests(unittest.TestCase):
    """The device line the pose ack is built from."""

    LINE = (GESTURE_ANGLE_PREFIX + " thumb=12 index=0 middle=45 ring=101 "
            "pinky=100 wrist=33;")

    def test_parses_every_joint_including_the_status_codes(self):
        self.assertEqual(
            parse_gesture_angles(self.LINE),
            {"thumb": 12, "index": 0, "middle": 45, "ring": 101,
             "pinky": 100, "wrist": 33},
        )

    def test_covers_exactly_the_joints_the_receiver_reports(self):
        self.assertEqual(set(parse_gesture_angles(self.LINE)), set(JOINTS))

    def test_finds_the_line_inside_surrounding_device_chatter(self):
        noisy = "\n".join(["[GestureController] targets: 11->204.96",
                           self.LINE, "GESTURE_RESULT: reached=1"])
        self.assertEqual(parse_gesture_angles(noisy)["wrist"], 33)

    def test_unrelated_or_truncated_input_yields_no_pose_rather_than_raising(self):
        # A poller on the hot path must not take an exception from a dropped
        # USB frame; a missing key is the easier failure to handle.
        for text in ("", None, "OK: gesture_angle index:35.0;",
                     GESTURE_ANGLE_PREFIX + " thumb= index=7 =3 middle"):
            parsed = parse_gesture_angles(text)
            self.assertIsInstance(parsed, dict)
        self.assertEqual(
            parse_gesture_angles(GESTURE_ANGLE_PREFIX + " thumb= index=7 =3"),
            {"index": 7},
        )


class PoseAckFrameTests(unittest.TestCase):
    """The packed frame that carries the pose upstream."""

    POSE = {"thumb": 0, "index": 25, "middle": 50, "ring": 75,
            "pinky": 100, "wrist": 101}

    def test_round_trips_value_and_every_joint(self):
        frame = pack_pose_ack(-3, JOINTS, self.POSE)
        value, pose = unpack_pose_ack(frame)
        self.assertEqual(value, -3)
        self.assertEqual(pose, self.POSE)

    def test_frame_is_fixed_width_and_magic_prefixed(self):
        frame = pack_pose_ack(2, JOINTS, self.POSE)
        self.assertTrue(frame.startswith(POSE_ACK_MAGIC))
        # magic(4) + int16 value + uint8 count + one byte per joint
        self.assertEqual(len(frame), 4 + 2 + 1 + len(JOINTS))

    def test_missing_joints_report_unavailable_not_zero(self):
        # 0 is a real position (home); a joint the device never mentioned is
        # not at home, it is unknown, and the two must not be conflated.
        _, pose = unpack_pose_ack(pack_pose_ack(1, JOINTS, {"index": 40}))
        self.assertEqual(pose["index"], 40)
        self.assertEqual(pose["thumb"], POSE_UNAVAILABLE)

    def test_rejects_text_and_truncated_frames(self):
        for bad in (b"", b"2", b"GESTURE_RESULT: reached=1",
                    pack_pose_ack(1, JOINTS, self.POSE)[:-2]):
            self.assertIsNone(unpack_pose_ack(bad), bad)

    def test_ascii_integer_ack_is_not_mistaken_for_a_pose_frame(self):
        for value in (0, 7, -7, 1000, 10004, -10004):
            self.assertFalse(str(value).encode().startswith(POSE_ACK_MAGIC))


class PoseAckFlowTests(unittest.TestCase):
    """Pose queries ride along with commands without disturbing the acks."""

    POSE_LINE = (GESTURE_ANGLE_PREFIX + " thumb=1 index=2 middle=3 ring=4 "
                 "pinky=5 wrist=6")

    def test_query_is_appended_after_the_command_not_before_it(self):
        # Ordering matters: the query must never delay the move it reports on.
        receiver = _receiver([])
        receiver.handle("2", "127.0.0.1:1")
        self.assertEqual(receiver.comm.sent[-1], POSE_QUERY)
        self.assertEqual(len(receiver.comm.sent), 2)
        self.assertEqual(receiver._pending[0], [2, 2])

    def test_no_query_is_sent_when_pose_acks_are_off(self):
        # Asserts against COMMAND_MAP rather than a literal, so retuning the
        # map (set_gesture vs set_gesture_angle, percentages) does not fail it.
        receiver = _receiver([], pose_ack=False)
        receiver.handle("2", "127.0.0.1:1")
        self.assertEqual(receiver.comm.sent, [COMMAND_MAP[2]])
        self.assertEqual(receiver._pending[0], [2, 1])

    def test_ack_carries_the_pose_and_the_integer_ack_is_unchanged(self):
        receiver = _receiver(["OK: gesture_angle index:85.0", self.POSE_LINE])
        sent = []
        receiver.send_upstream = lambda v: (sent.append(("int", int(v))), True)[1]
        receiver.sock = None      # send_pose_upstream needs a socket; capture instead
        receiver.handle("2", "127.0.0.1:1")
        receiver.send_pose_upstream = lambda v: (
            sent.append(("pose", pack_pose_ack(v, JOINTS, receiver.pose))), True
        )[1]
        receiver.drain_replies()

        self.assertEqual(len(receiver._pending), 0, "both replies accounted for")
        self.assertEqual([kind for kind, _ in sent], ["int", "pose"])
        self.assertEqual(sent[0][1], 2)
        self.assertEqual(unpack_pose_ack(sent[1][1])[1]["wrist"], 6)

    def test_pose_reply_retires_its_own_slot_unlike_a_move_outcome(self):
        # The pose answers a query we sent, so it is solicited and must retire
        # one slot. GESTURE_RESULT is unsolicited and must not.
        receiver = _receiver(["OK: gesture_angle index:85.0",
                              "GESTURE_RESULT: reached=1 stalled=0",
                              self.POSE_LINE])
        acked = []
        receiver.send_upstream = lambda v: (acked.append(int(v)), True)[1]
        receiver.send_pose_upstream = lambda v: True
        receiver.send_text_upstream = lambda t: True
        receiver.handle("2", "127.0.0.1:1")
        receiver.drain_replies()
        self.assertEqual(acked, [2])
        self.assertEqual(receiver.outcomes, 1)

    def test_silent_firmware_disables_pose_acks_instead_of_stalling_them(self):
        # An unknown command is SILENT in firmware. Left enabled, every command
        # would wait forever on a reply that never comes.
        class _Older(MockComm):
            @staticmethod
            def _reply_for(command):
                if command.startswith("get_gesture_angle"):
                    return None
                return MockComm._reply_for(command)

        comm = _Older(latency_ms=0.0, log=False)
        comm.connect()
        receiver = Receiver(comm, COMMAND_MAP, echo_replies=False, verbose=False)
        self.assertFalse(receiver.probe_pose_support(timeout=0.05))
        self.assertFalse(receiver.pose_ack)

        receiver.return_addr = ("127.0.0.1", 10004)
        acked = []
        receiver.send_upstream = lambda v: (acked.append(int(v)), True)[1]
        receiver.handle("2", "127.0.0.1:1")
        for _ in range(3):
            receiver.drain_replies()
        self.assertEqual(acked, [2], "acks still flow on older firmware")

    def test_probe_enables_pose_acks_against_a_device_that_answers(self):
        comm = MockComm(latency_ms=0.0, log=False)
        comm.connect()
        receiver = Receiver(comm, COMMAND_MAP, echo_replies=False, verbose=False)
        self.assertTrue(receiver.probe_pose_support(timeout=0.5))
        self.assertTrue(receiver.pose_ack)
        self.assertEqual(set(receiver.pose), set(JOINTS))


if __name__ == "__main__":
    unittest.main()
