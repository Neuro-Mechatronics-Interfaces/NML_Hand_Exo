# LSL streaming

These examples use the public LSL clients, subscribers, and plotting helpers.
Run them from the repository root.

```powershell
# Publish synthetic samples
python examples/06_lsl_streaming/LSL/lsl_broadcast_test.py --help

# Inspect visible streams
python examples/06_lsl_streaming/LSL/lsl_subscribe_test.py

# Listen to markers without moving hardware
python examples/06_lsl_streaming/LSL/lsl_gesture_sub.py --help

# Bridge gesture markers to an exo
python examples/06_lsl_streaming/LSL/lsl_gesture_controller.py --help

# Plot EMG
python examples/06_lsl_streaming/LSL/lsl_stacked_plot.py --help
python examples/06_lsl_streaming/LSL/lsl_grid_plot.py
python examples/06_lsl_streaming/LSL/lsl_rms_barplot.py
```

The classifier and fixed-threshold control prototypes were removed because
their model contract and safety behavior did not match the maintained
`handexo emg-intent` workflow.
