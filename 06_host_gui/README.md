# 06_host_gui (G3)

Python host GUI (PyQt5 + pyserial).

To start: copy `00_baseline/sample_code/Micro Arm GUI.py` to `micro_arm_gui.py`, commit it unchanged, then extend it.

Requirements from the brief: port + baud selection with connect/disconnect/reconnect, named sliders using calibrated limits, send-on-release or rate limit, clear HOME/STOP (text + keyboard, not colour only), reply/state display, timestamped log file, graceful port errors.

```
pip install PyQt5 pyserial
```

On macOS, Nano clones (CH340) usually appear as `/dev/cu.wchusbserial*` or `/dev/cu.usbserial*`.
