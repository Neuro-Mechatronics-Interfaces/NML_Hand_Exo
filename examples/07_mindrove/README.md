# MindRove integration

These are optional live-hardware applications that use the third-party
`mindrove` SDK directly. They do not require another NML repository.

```powershell
python -m pip install -e ".[integrations]"
python examples/07_mindrove/MindRoveExoControlPanel.py
```

`MindRoveExoDemo_4_10.py` contains the lower-level acquisition and classifier
implementation used by the control panel. Treat both as hardware integration
examples; the maintained session-based decoder is `handexo emg-intent`.
