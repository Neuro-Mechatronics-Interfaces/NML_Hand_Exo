import json, threading
from queue import Queue, Empty
from pynput.keyboard import Key, KeyCode, Controller
from pylsl import resolve_byprop, StreamInlet

keyboard_ctl = Controller()
event_q = Queue()

SPECIALS = {
    "SPACE": Key.space, "ENTER": Key.enter, "RETURN": Key.enter,
    "LEFT": Key.left, "RIGHT": Key.right, "UP": Key.up, "DOWN": Key.down,
    "ESC": Key.esc, "TAB": Key.tab, "SHIFT": Key.shift, "CTRL": Key.ctrl, "ALT": Key.alt
}

def to_key_obj(name):
    if name in SPECIALS:
        return SPECIALS[name]

    if len(name) == 1:
        return KeyCode.from_char(name.lower())

    if len(name) > 1 and name.isalpha():
        return KeyCode.from_char(name[0].lower())
    return None

def make_reader(stream_name):
    print(f"Resolving stream: {stream_name} ...")
    streams = resolve_byprop('name', stream_name, minimum=1, timeout=10.0) 
    if not streams:
        print(f"WARNING: stream '{stream_name}' not found.")
        return None
    
    inlet = StreamInlet(streams[0], max_buflen=60)
    print(f"Connected to '{stream_name}'.")
    def loop():
        while True:
            sample, ts = inlet.pull_sample(timeout=0.1)
            if not sample: 
                continue
            try:
                evt = json.loads(sample[0])
                event_q.put(evt)
            except Exception:
                pass
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return inlet


# STREAMS = ["KeyEvents-Mac", "KeyEvents-Alienware", "KeyEvents-Mac-2"]
STREAMS = ["KeyEvents-Mac-2", "KeyEvents-Alienware"]
for name in STREAMS:
    make_reader(name)

print("Receiver running. Focus the emulator window and leave this terminal in the background.")

while True:
    try:
        evt = event_q.get(timeout=1.0)
        keyobj = to_key_obj(evt.get("key",""))
        if not keyobj: 
            continue
        if evt.get("state") == "down":
            keyboard_ctl.press(keyobj)
        elif evt.get("state") == "up":
            keyboard_ctl.release(keyobj)
    except Empty:
        pass