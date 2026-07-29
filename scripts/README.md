# Scripts #
This folder contains one-off scripts implementing ad hoc functionality without a GUI. 

## UDP Forwarder ##
Usage (from `.handexo` virtual environment): 
```bash
python scripts/udp_gesture_receiver.py --port 10003
```
Forwards signed integers as corresponding position-mapped digit flexion/extension commands. "Rest" (0) maps to `set_gesture:grasp:open`. 