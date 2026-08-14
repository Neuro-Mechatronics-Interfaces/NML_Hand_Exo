# Advanced firmware configuration

`example_advanced_config.py` demonstrates debug output, OLED state, motor
operating modes, exoskeleton modes, baud queries, and device information:

```powershell
python examples/04_advanced/example_advanced_config.py
```

The port is configured in the script. The example changes live firmware modes;
run it only on a bench device. The former unrelated Uno/Pico UART prototype was
removed because it depended on an unavailable local `usbserialreader` module.
