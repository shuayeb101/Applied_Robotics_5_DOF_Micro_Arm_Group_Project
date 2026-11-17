# 06_host_gui (G3)

Python host GUI.

- `micro_arm_gui.py` — MicroArm control GUI v1.0.0 (Shuayeb), for `05_firmware/MicroArm_Firmware`. Uses tkinter (built in), so PyQt5 isn't needed
- `serial/` — pyserial 3.5, bundled unmodified (BSD-3-Clause, see `serial/LICENSE_pyserial.txt`) so the GUI runs on locked-down uni PCs without installing anything

Run from this folder: `python micro_arm_gui.py` (Windows: `py micro_arm_gui.py`). Logs go to `logs/`, which git ignores. Full guide: `05_firmware/MicroArm_v1.0_README.md`.

Requirements from the brief: port + baud selection with connect/disconnect/reconnect, named sliders using calibrated limits, send-on-release or rate limit, clear HOME/STOP (text + keyboard, not colour only), reply/state display, timestamped log file, graceful port errors.



On macOS, Nano clones (CH340) usually appear as `/dev/cu.wchusbserial*` or `/dev/cu.usbserial*`.
