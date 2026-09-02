import importlib.util
import pathlib
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).parents[1]
    / "examples"
    / "08_udp"
    / "continuous_udp_receiver.py"
)
SPEC = importlib.util.spec_from_file_location("continuous_udp_receiver", MODULE_PATH)
receiver_module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(receiver_module)


class FakeComm:
    def __init__(self):
        self.sent = []

    def send(self, message):
        self.sent.append(message)


class ReceiverSafetyTests(unittest.TestCase):
    def make_receiver(self):
        comm = FakeComm()
        receiver = receiver_module.Receiver(comm, verbose=False)
        return receiver, comm

    def test_full_ack_queue_drops_new_frame_before_dispatch(self):
        receiver, comm = self.make_receiver()
        for sequence in range(receiver_module.MAX_PENDING_ACKS):
            receiver._pending.append([sequence, {}, ("127.0.0.1", 1234)])

        applied = receiver._apply(
            {joint: 0 for joint in receiver_module.JOINTS}, sequence=999
        )

        self.assertFalse(applied)
        self.assertEqual(comm.sent, [])
        self.assertEqual(len(receiver._pending), receiver_module.MAX_PENDING_ACKS)
        self.assertEqual(receiver.dropped_acks, 1)

    def test_reconnect_discards_old_reply_correlation(self):
        receiver, _ = self.make_receiver()
        receiver._pending.append([1, {}, ("127.0.0.1", 1234)])
        receiver._ack_addr = ("127.0.0.1", 1234)

        # The cleanup occurs before any port discovery attempt. Use zero
        # attempts so this test never touches hardware or sleeps.
        self.assertFalse(receiver.reconnect(attempts=0))
        self.assertEqual(list(receiver._pending), [])
        self.assertIsNone(receiver._ack_addr)


class ActiveSideTests(unittest.TestCase):
    class ReplyComm:
        def __init__(self, reply):
            self.reply = reply

        def flush_input(self):
            pass

        def send(self, _message):
            pass

        def receive(self, wait_until_return=False, timeout=None):
            reply, self.reply = self.reply, ""
            return reply

    def test_refuses_both_connected_hands(self):
        comm = self.ReplyComm("Motor status: {active_side: both};")
        with self.assertRaisesRegex(RuntimeError, "both left and right"):
            receiver_module.read_active_side(comm)

    def test_accepts_one_connected_hand(self):
        comm = self.ReplyComm("Motor status: {active_side: right};")
        self.assertEqual(receiver_module.read_active_side(comm), "right")


if __name__ == "__main__":
    unittest.main()
