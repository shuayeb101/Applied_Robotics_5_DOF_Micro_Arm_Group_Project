#!/usr/bin/env python3
"""
CT6056 5-DOF Micro Arm - host control GUI                          v1.0.0

Talks to MicroArm_Firmware.ino over USB serial (115200 baud, newline-terminated).

Runs on Windows / macOS / Linux with Python 3.8+.
Needs only the standard library (tkinter) plus pyserial. A copy of pyserial is
bundled in this folder, so nothing has to be installed on a locked-down PC.

Run:   python micro_arm_gui.py        (Windows: py micro_arm_gui.py)

Keyboard:  Esc = STOP    Ctrl+H = HOME    Ctrl+R = RESET FAULT    F5 = STATUS
"""

import os
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)  # use the bundled pyserial if it is not installed
try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    print("pyserial not found. Keep the 'serial' folder next to this script, "
          "or run:  py -m pip install --user pyserial")
    raise

APP_VERSION = "1.0.0"
BAUD_RATES = ["115200", "57600", "38400", "19200", "9600"]
STATUS_PERIOD_MS = 400       # STATUS poll = heartbeat for the firmware watchdog
WATCHDOG_MS = 1500           # firmware stops motion if no line arrives for this long
LINK_STALE_S = 1.5           # GUI flags the link if no reply for this long

# Defaults until the controller sends CONFIG (kept identical to the firmware table)
DEFAULT_JOINTS = [
    {"name": "BASE",  "label": "Base",           "ch": 0, "lo": 0.0,  "hi": 180.0, "home": 90.0},
    {"name": "SWING", "label": "Swing (shoulder)", "ch": 1, "lo": 10.0, "hi": 170.0, "home": 90.0},
    {"name": "ARM1",  "label": "Arm One (elbow)",  "ch": 2, "lo": 0.0,  "hi": 180.0, "home": 90.0},
    {"name": "ARM2",  "label": "Arm Two (wrist)",  "ch": 3, "lo": 0.0,  "hi": 180.0, "home": 90.0},
    {"name": "GRIP",  "label": "Gripper",          "ch": 4, "lo": 30.0, "hi": 150.0, "home": 90.0},
]
STATE_STYLE = {   # text is always shown; colour is only a secondary cue
    "INIT":   ("INIT - outputs off, press HOME", "#6b6b70"),
    "IDLE":   ("IDLE - ready", "#1f7a3a"),
    "MOVING": ("MOVING", "#2457c5"),
    "HOMING": ("HOMING", "#2457c5"),
    "HOLD":   ("HOLD - stopped, holding position", "#a15c00"),
    "FAULT":  ("FAULT - see reason, then RESET FAULT", "#b3261e"),
    "OFFLINE": ("NOT CONNECTED", "#6b6b70"),
}


def parse_kv(line):
    """'STATUS state=IDLE pos=1,2 ...' -> dict (values kept as strings)."""
    out = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


class SerialLink:
    """Owns the port. A background thread reads lines into a queue; writes are locked."""

    def __init__(self, on_line, on_drop):
        self.ser = None
        self.rx = queue.Queue()
        self.on_line, self.on_drop = on_line, on_drop
        self._lock = threading.Lock()
        self._alive = False

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud):
        self.ser = serial.Serial(port, int(baud), timeout=0.1, write_timeout=0.5)
        self._alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def close(self):
        self._alive = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def write_line(self, text):
        if not self.is_open:
            raise IOError("port not open")
        with self._lock:
            self.ser.write((text + "\n").encode("ascii", "replace"))

    def _reader(self):
        buf = b""
        while self._alive:
            try:
                chunk = self.ser.read(256)
            except Exception as e:  # unplugged / port vanished
                if self._alive:
                    self.rx.put(("DROP", str(e)))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.rx.put(("LINE", raw.decode("ascii", "replace").strip()))


class ArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"Micro Arm Control  v{APP_VERSION}")
        self.geometry("1280x780")
        self.minsize(980, 640)

        self.link = SerialLink(None, None)
        self.seq = 0
        self.pending = {}            # seq -> (command, time sent)
        self.joints = [dict(j) for j in DEFAULT_JOINTS]
        self.dragging = [False] * 5
        self.last_rx_time = 0.0
        self.state = "OFFLINE"
        self.config_lines = []
        self.status_job = None

        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        self.log_path = os.path.join(HERE, "logs", datetime.now().strftime("microarm_%Y%m%d_%H%M%S.log"))
        self.log_file = open(self.log_path, "a", encoding="utf-8")

        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.log("INFO", f"GUI v{APP_VERSION} started. Log file: {self.log_path}")
        self.after(50, self._pump)
        self.after(500, self._link_watch)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Big.TButton", font=("Segoe UI", 13, "bold"), padding=10)
        style.configure("Stop.TButton", font=("Segoe UI", 14, "bold"), padding=12,
                        foreground="white", background="#b3261e")
        style.map("Stop.TButton", background=[("active", "#8c1d17")])
        style.configure("Head.TLabel", font=("Segoe UI", 11, "bold"))

        # --- connection bar
        bar = ttk.Frame(self, padding=(10, 8))
        bar.pack(fill="x")
        ttk.Label(bar, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=34)
        self.port_box.pack(side="left", padx=(4, 4))
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Label(bar, text="  Baud").pack(side="left")
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self.baud_var, values=BAUD_RATES, width=8,
                     state="readonly").pack(side="left", padx=4)
        self.conn_btn = ttk.Button(bar, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=8)
        self.conn_lbl = ttk.Label(bar, text="Disconnected")
        self.conn_lbl.pack(side="left", padx=6)

        body = ttk.Frame(self, padding=(10, 0, 10, 10))
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")

        # --- safety buttons
        safety = ttk.Frame(left)
        safety.pack(fill="x", pady=(0, 8))
        ttk.Button(safety, text="■  STOP  (Esc)", style="Stop.TButton",
                   command=self.cmd_stop).pack(side="left", fill="x", expand=True)
        ttk.Button(safety, text="⌂  HOME  (Ctrl+H)", style="Big.TButton",
                   command=self.cmd_home).pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(safety, text="RESET FAULT (Ctrl+R)", style="Big.TButton",
                   command=lambda: self.send("RESET_FAULT")).pack(side="left", fill="x", expand=True)

        # --- state banner
        self.state_lbl = tk.Label(left, text="NOT CONNECTED", font=("Segoe UI", 14, "bold"),
                                  fg="white", bg="#6b6b70", anchor="w", padx=10, pady=6)
        self.state_lbl.pack(fill="x")
        self.fault_lbl = ttk.Label(left, text="Last error: -", foreground="#b3261e")
        self.fault_lbl.pack(fill="x", pady=(4, 8))

        # --- joints
        jf = ttk.LabelFrame(left, text="Joints  (servo degrees; drag and release, or type a value and press Enter)", padding=8)
        jf.pack(fill="x")
        ttk.Label(jf, text="Joint", style="Head.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(jf, text="Target", style="Head.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(jf, text="Value", style="Head.TLabel").grid(row=0, column=2)
        ttk.Label(jf, text="Actual", style="Head.TLabel").grid(row=0, column=4)
        jf.columnconfigure(1, weight=1)
        self.scales, self.entries, self.actual_lbls, self.limit_lbls, self.name_lbls = [], [], [], [], []
        for i, j in enumerate(self.joints):
            name = ttk.Label(jf, text=f"Ch {j['ch']}  {j['label']}")
            name.grid(row=1 + 2 * i, column=0, sticky="w", padx=(0, 8))
            sc = ttk.Scale(jf, from_=j["lo"], to=j["hi"], orient="horizontal")
            sc.set(j["home"])
            sc.grid(row=1 + 2 * i, column=1, sticky="ew")
            sc.bind("<ButtonPress-1>", lambda e, k=i: self._drag(k, True))
            sc.bind("<ButtonRelease-1>", lambda e, k=i: self._release(k))
            sc.configure(command=lambda v, k=i: self._scale_moved(k, v))
            ent = ttk.Spinbox(jf, from_=j["lo"], to=j["hi"], increment=1, width=7)
            ent.set(f"{j['home']:.0f}")
            ent.grid(row=1 + 2 * i, column=2, padx=6)
            ent.bind("<Return>", lambda e, k=i: self._send_entry(k))
            ttk.Button(jf, text="Go", width=4, command=lambda k=i: self._send_entry(k)).grid(row=1 + 2 * i, column=3)
            act = ttk.Label(jf, text="--", width=8, anchor="e")
            act.grid(row=1 + 2 * i, column=4, padx=(8, 0))
            lim = ttk.Label(jf, text=f"limits {j['lo']:.0f}-{j['hi']:.0f}, home {j['home']:.0f}",
                            foreground="#6b6b70", font=("Segoe UI", 8))
            lim.grid(row=2 + 2 * i, column=1, sticky="w", pady=(0, 4))
            self.scales.append(sc); self.entries.append(ent); self.actual_lbls.append(act)
            self.limit_lbls.append(lim); self.name_lbls.append(name)

        # --- speed + XY
        row = ttk.Frame(left)
        row.pack(fill="x", pady=8)
        sp = ttk.LabelFrame(row, text="Speed (deg/s, 5-180)", padding=8)
        sp.pack(side="left", fill="both", expand=True)
        self.speed_var = tk.StringVar(value="60")
        ttk.Spinbox(sp, from_=5, to=180, increment=5, textvariable=self.speed_var, width=6).pack(side="left")
        ttk.Button(sp, text="Set", command=self.cmd_speed).pack(side="left", padx=6)
        self.speed_lbl = ttk.Label(sp, text="active: --")
        self.speed_lbl.pack(side="left", padx=6)

        xy = ttk.LabelFrame(left, text="XY target (mm, shoulder pivot = origin, x forward, y up)", padding=8)
        xy.pack(fill="x")
        self.x_var, self.y_var, self.br_var = tk.StringVar(value="70"), tk.StringVar(value="40"), tk.StringVar(value="UP")
        ttk.Label(xy, text="x").pack(side="left")
        ttk.Entry(xy, textvariable=self.x_var, width=6).pack(side="left", padx=(2, 6))
        ttk.Label(xy, text="y").pack(side="left")
        ttk.Entry(xy, textvariable=self.y_var, width=6).pack(side="left", padx=(2, 6))
        ttk.Radiobutton(xy, text="Elbow up", value="UP", variable=self.br_var).pack(side="left")
        ttk.Radiobutton(xy, text="Elbow down", value="DOWN", variable=self.br_var).pack(side="left")
        ttk.Button(xy, text="Send XY", command=self.cmd_xy).pack(side="left", padx=6)

        # --- status panel (right)
        st = ttk.LabelFrame(right, text="Controller status", padding=8)
        st.pack(fill="x")
        self.status_vals = {}
        for r, (k, label) in enumerate([("state", "State"), ("pos", "Position"), ("tgt", "Target"),
                                        ("en", "Powered joints"), ("spd", "Speed"), ("wd", "Watchdog"),
                                        ("err", "Last error"), ("fault", "Fault"), ("link", "Link")]):
            ttk.Label(st, text=label + ":").grid(row=r, column=0, sticky="w")
            v = ttk.Label(st, text="--")
            v.grid(row=r, column=1, sticky="w", padx=6)
            self.status_vals[k] = v

        # --- manual command line (for protocol tests)
        mf = ttk.LabelFrame(right, text="Manual command (for protocol / fault tests)", padding=8)
        mf.pack(fill="x", pady=8)
        self.manual_var = tk.StringVar()
        me = ttk.Entry(mf, textvariable=self.manual_var)
        me.pack(side="left", fill="x", expand=True)
        me.bind("<Return>", lambda e: self.cmd_manual())
        ttk.Button(mf, text="Send", command=self.cmd_manual).pack(side="left", padx=4)
        ttk.Button(mf, text="STATUS (F5)", command=lambda: self.send("STATUS")).pack(side="left")

        # --- log
        lf = ttk.LabelFrame(right, text="Log (also saved to logs\\ folder)", padding=4)
        lf.pack(fill="both", expand=True)
        self.show_status = tk.BooleanVar(value=False)
        ttk.Checkbutton(lf, text="Show periodic STATUS lines", variable=self.show_status).pack(anchor="w")
        self.log_txt = tk.Text(lf, height=12, wrap="none", font=("Consolas", 9))
        ys = ttk.Scrollbar(lf, command=self.log_txt.yview)
        self.log_txt.configure(yscrollcommand=ys.set)
        ys.pack(side="right", fill="y")
        self.log_txt.pack(fill="both", expand=True)
        for tag, col in [("TX", "#2457c5"), ("ERR", "#b3261e"), ("EVENT", "#a15c00"), ("INFO", "#6b6b70"),
                         ("ACK", "#1f7a3a"), ("DONE", "#1f7a3a")]:
            self.log_txt.tag_configure(tag, foreground=col)
        ttk.Button(lf, text="Open log folder", command=self.open_logs).pack(anchor="e", pady=(4, 0))

    def _bind_keys(self):
        self.bind_all("<Escape>", lambda e: self.cmd_stop())
        self.bind_all("<Control-h>", lambda e: self.cmd_home())
        self.bind_all("<Control-r>", lambda e: self.send("RESET_FAULT"))
        self.bind_all("<F5>", lambda e: self.send("STATUS"))

    # ------------------------------------------------------------ logging
    def log(self, kind, text, quiet=False):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{stamp}  {kind:<5} {text}"
        self.log_file.write(f"{datetime.now().isoformat(timespec='milliseconds')}  {kind:<5} {text}\n")
        self.log_file.flush()
        if quiet:
            return
        self.log_txt.insert("end", line + "\n", kind)
        if int(self.log_txt.index("end-1c").split(".")[0]) > 3000:
            self.log_txt.delete("1.0", "500.0")
        self.log_txt.see("end")

    def open_logs(self):
        path = os.path.join(HERE, "logs")
        try:
            if sys.platform.startswith("win"):
                os.startfile(path)  # noqa
            elif sys.platform == "darwin":
                os.system(f'open "{path}"')
            else:
                os.system(f'xdg-open "{path}"')
        except Exception as e:
            self.log("ERR", f"could not open folder: {e}")

    # --------------------------------------------------------- connection
    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        items = [f"{p.device} - {p.description}" for p in ports]
        self.port_box["values"] = items
        if items and not self.port_var.get():
            # prefer something that looks like an Arduino / USB-serial adapter
            pick = next((s for s in items if any(k in s.upper() for k in
                         ("ARDUINO", "CH340", "CH341", "USB-SERIAL", "USB SERIAL", "FTDI", "WCH", "USBSERIAL"))), items[0])
            self.port_var.set(pick)
        self.log("INFO", f"ports found: {', '.join(p.device for p in ports) or 'none'}")

    def toggle_connect(self):
        if self.link.is_open:
            self.disconnect("user")
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().split(" - ")[0].strip()
        if not port:
            self.log("ERR", "no port selected - plug in the Nano and press Refresh")
            return
        try:
            self.link.open(port, self.baud_var.get())
        except serial.SerialException as e:
            msg = str(e)
            hint = ""
            if "denied" in msg.lower() or "busy" in msg.lower() or "PermissionError" in msg:
                hint = " (port busy - close the Arduino Serial Monitor / other programs)"
            elif "could not open" in msg.lower() or "FileNotFound" in msg:
                hint = " (port not found - check the cable and press Refresh)"
            self.log("ERR", f"cannot open {port}: {msg}{hint}")
            self.conn_lbl.configure(text="Connect failed")
            return
        self.conn_btn.configure(text="Disconnect")
        self.conn_lbl.configure(text=f"Connected {port} @ {self.baud_var.get()} - waiting for READY...")
        self.log("INFO", f"opened {port}. Opening the port resets the Nano; it restarts with outputs OFF.")
        self.last_rx_time = time.time()
        self.pending.clear()
        # the Nano resets on connect: give the bootloader ~2 s before talking
        self.after(2200, self._after_connect)

    def _after_connect(self):
        if not self.link.is_open:
            return
        self.config_lines = []
        self.send("CONFIG")
        self.send(f"WATCHDOG {WATCHDOG_MS}")
        self.send("STATUS")
        self._schedule_status()

    def disconnect(self, why):
        if self.status_job:
            self.after_cancel(self.status_job)
            self.status_job = None
        self.link.close()
        self.conn_btn.configure(text="Connect")
        self.conn_lbl.configure(text=f"Disconnected ({why})")
        self.set_state("OFFLINE")
        for k, v in self.status_vals.items():   # don't show stale controller data
            if k != "link":
                v.configure(text="--")
        for lbl in self.actual_lbls:
            lbl.configure(text="--")
        self.log("INFO" if why == "user" else "ERR", f"disconnected: {why}")

    def _schedule_status(self):
        if self.link.is_open:
            self.send("STATUS", quiet=True)
            self.status_job = self.after(STATUS_PERIOD_MS, self._schedule_status)

    def _link_watch(self):
        if self.link.is_open:
            age = time.time() - self.last_rx_time
            self.status_vals["link"].configure(
                text=f"OK (last reply {age:.1f}s ago)" if age < LINK_STALE_S else f"NO REPLY for {age:.1f}s",
                foreground="#1f7a3a" if age < LINK_STALE_S else "#b3261e")
        else:
            self.status_vals["link"].configure(text="offline", foreground="#6b6b70")
        self.after(500, self._link_watch)

    # ------------------------------------------------------------- sending
    def send(self, cmd, quiet=False):
        if not self.link.is_open:
            self.log("ERR", f"not connected - '{cmd}' not sent")
            return False
        self.seq = (self.seq % 99999) + 1
        line = f"#{self.seq} {cmd}"
        try:
            self.link.write_line(line)
        except Exception as e:
            self.log("ERR", f"write failed: {e}")
            self.disconnect("write error / cable unplugged")
            return False
        self.pending[str(self.seq)] = (cmd, time.time())
        self.log("TX", line, quiet=quiet and not self.show_status.get())
        return True

    def cmd_stop(self):
        self.send("STOP")

    def cmd_home(self):
        self.send("HOME")

    def cmd_speed(self):
        try:
            v = float(self.speed_var.get())
        except ValueError:
            self.log("ERR", "speed must be a number")
            return
        if not 5 <= v <= 180:
            self.log("ERR", "speed must be between 5 and 180 deg/s")
            return
        self.send(f"SPEED {v:g}")

    def cmd_xy(self):
        try:
            x, y = float(self.x_var.get()), float(self.y_var.get())
        except ValueError:
            self.log("ERR", "x and y must be numbers (mm)")
            return
        self.send(f"XY {x:g} {y:g} {self.br_var.get()}")

    def cmd_manual(self):
        cmd = self.manual_var.get().strip()
        if not cmd:
            return
        if not self.link.is_open:
            self.log("ERR", "not connected")
            return
        # sent raw (no host validation) so malformed-input tests reach the controller
        try:
            self.link.write_line(cmd)
            self.log("TX", cmd + "   (raw, no host validation)")
        except Exception as e:
            self.log("ERR", f"write failed: {e}")
        self.manual_var.set("")

    # joint controls: send-on-release, so dragging can never flood the controller
    def _drag(self, k, on):
        self.dragging[k] = on

    def _scale_moved(self, k, v):
        self.entries[k].set(f"{float(v):.0f}")

    def _release(self, k):
        self.dragging[k] = False
        self._send_joint(k, round(float(self.scales[k].get())))

    def _send_entry(self, k):
        try:
            v = float(self.entries[k].get())
        except ValueError:
            self.log("ERR", f"{self.joints[k]['label']}: value must be a number")
            return
        self.scales[k].set(v)
        self._send_joint(k, v)

    def _send_joint(self, k, v):
        j = self.joints[k]
        if not j["lo"] <= v <= j["hi"]:
            self.log("ERR", f"{j['label']}: {v:g} is outside {j['lo']:g}-{j['hi']:g} (not sent)")
            return
        self.send(f"S {k} {v:g}")

    # ------------------------------------------------------------ receiving
    def _pump(self):
        try:
            while True:
                kind, payload = self.link.rx.get_nowait()
                if kind == "DROP":
                    self.disconnect(f"port lost ({payload})")
                else:
                    self.handle_line(payload)
        except queue.Empty:
            pass
        self.after(30, self._pump)

    def handle_line(self, line):
        if not line:
            return
        self.last_rx_time = time.time()
        head = line.split()[0]
        seq = None
        parts = line.split()
        if len(parts) > 1 and parts[1].startswith("#"):
            seq = parts[1][1:]
        is_status = head == "STATUS"
        tag = head if head in ("ERR", "EVENT", "ACK", "DONE") else "RX"
        self.log(tag, line, quiet=is_status and not self.show_status.get())
        if seq and head in ("ACK", "ERR", "STATUS"):
            self.pending.pop(seq, None)

        if head == "READY":
            self.conn_lbl.configure(text=self.conn_lbl.cget("text").replace(" - waiting for READY...", "") + " - controller ready")
            kv = parse_kv(line)
            if "state" in kv:
                self.set_state(kv["state"])
        elif is_status:
            self.apply_status(parse_kv(line))
        elif head == "CONFIG":
            self.apply_config(line)
        elif head == "ERR":
            self.fault_lbl.configure(text="Last error: " + " ".join(parts[1:]))
        elif head == "EVENT" and "FAULT" in line:
            self.fault_lbl.configure(text="FAULT: " + " ".join(parts[1:]))
        elif head == "ACK" and "SPEED" in line:
            self.speed_lbl.configure(text="active: " + parse_kv(line).get("speed", "?"))

    def apply_config(self, line):
        kv = parse_kv(line)
        if line.startswith("CONFIG END"):
            self.log("INFO", "joint limits loaded from controller")
            return
        if "joint" not in kv:
            return
        try:
            k = int(kv["joint"])
            lo, hi, home = float(kv["lo"]), float(kv["hi"]), float(kv["home"])
        except (KeyError, ValueError):
            return
        if not 0 <= k < 5:
            return
        j = self.joints[k]
        j.update(lo=lo, hi=hi, home=home, ch=int(kv.get("ch", j["ch"])))
        self.scales[k].configure(from_=lo, to=hi)
        self.entries[k].configure(from_=lo, to=hi)
        self.limit_lbls[k].configure(text=f"limits {lo:g}-{hi:g}, home {home:g}  (controller)")
        self.name_lbls[k].configure(text=f"Ch {j['ch']}  {j['label']}")

    def apply_status(self, kv):
        if "waiting for READY" in self.conn_lbl.cget("text"):
            self.conn_lbl.configure(text=self.conn_lbl.cget("text").replace(" - waiting for READY...", " - controller responding"))
        st = kv.get("state", "?")
        self.set_state(st)
        for key in ("pos", "tgt", "en", "err", "fault"):
            if key in kv:
                self.status_vals[key].configure(text=kv[key].replace(",", ",  "))
        self.status_vals["state"].configure(text=st)
        if "spd" in kv:
            self.status_vals["spd"].configure(text=kv["spd"] + " deg/s")
            self.speed_lbl.configure(text="active: " + kv["spd"])
        if "wd" in kv:
            self.status_vals["wd"].configure(text="off" if kv["wd"] == "0" else kv["wd"] + " ms")
        try:
            pos = [float(v) for v in kv.get("pos", "").split(",")]
            tgt = [float(v) for v in kv.get("tgt", "").split(",")]
            en = kv.get("en", "11111")
        except ValueError:
            return
        for k in range(min(5, len(pos))):
            self.actual_lbls[k].configure(text=f"{pos[k]:.1f}°" if en[k:k + 1] == "1" else "off")
            # keep sliders in step with the controller, but never while the user drags
            if not self.dragging[k] and k < len(tgt) and self.focus_get() is not self.entries[k]:
                self.scales[k].set(tgt[k])
                self.entries[k].set(f"{tgt[k]:.0f}")

    def set_state(self, st):
        self.state = st
        text, col = STATE_STYLE.get(st, (st, "#6b6b70"))
        self.state_lbl.configure(text="State: " + text, bg=col)

    def on_close(self):
        try:
            if self.link.is_open:
                self.send("STOP")
                time.sleep(0.1)
        finally:
            self.link.close()
            self.log("INFO", "GUI closed")
            self.log_file.close()
            self.destroy()


if __name__ == "__main__":
    ArmGUI().mainloop()
