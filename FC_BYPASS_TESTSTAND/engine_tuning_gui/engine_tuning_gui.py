#!/usr/bin/env python3
"""Desktop tuning/telemetry GUI for the RP2040 custom-CAN engine controller.

The GUI talks to the companion RP2040 USB↔CAN bridge over USB CDC serial.
"""
from __future__ import annotations

import csv
import json
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

STATE_NAMES = {
    0: "DISARMED",
    1: "ARMED_WAIT_FOR_SPIN",
    2: "PRIMING_AFTER_SPIN",
    3: "IDLE_WAIT_FOR_ZERO",
    4: "RUNNING",
    5: "SERVO_TEST",
    6: "FAULT",
}

SETTINGS_FILE_NAME = "engine_gui_settings.json"
TELEMETRY_PERIOD_MIN_MS = 5
TELEMETRY_PERIOD_MAX_MS = 5000


@dataclass
class Telemetry:
    rpm: float = 0.0
    out_us: int = 0
    state: int = 0
    flags: int = 0
    target_rpm: float = 0.0
    ff_us: int = 0
    pid_us: float = 0.0
    temp_c: float = 0.0
    current_a: float = 0.0
    vbus_v: float = 0.0
    runtime_s: int = 0
    last_rx_monotonic: float = 0.0

    @property
    def armed_flag(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def command_link_flag(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def stationary_flag(self) -> bool:
        return bool(self.flags & 0x04)


class SerialWorker:
    def __init__(self, incoming: "queue.Queue[str]") -> None:
        self.incoming = incoming
        self.ser: Optional[serial.Serial] = None
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    def connect(self, port: str, baudrate: int = 115200) -> None:
        self.disconnect()
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05, write_timeout=0.2)
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.35)
        self.thread = None
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def write_line(self, line: str) -> None:
        if not self.is_connected():
            raise RuntimeError("Serial bridge is not connected")
        payload = (line.strip() + "\n").encode("ascii", errors="strict")
        with self.lock:
            assert self.ser is not None
            self.ser.write(payload)
            self.ser.flush()

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                if self.ser is None:
                    break
                raw = self.ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                if text:
                    self.incoming.put(text)
            except (serial.SerialException, OSError) as exc:
                self.incoming.put(f"__SERIAL_ERROR__ {exc}")
                break


class EngineGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Engine CAN Tuning Console")
        self.geometry("1320x860")
        self.minsize(1180, 760)

        self.telemetry = Telemetry()
        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.serial_worker = SerialWorker(self.rx_queue)
        self.settings_path = Path(__file__).with_name(SETTINGS_FILE_NAME)
        self.saved_settings = self._load_settings_file()

        self.command_armed = False
        self.manual_throttle = tk.DoubleVar(value=0.0)
        self.sweep_active = False
        self.sweep_start_time = 0.0
        self.sweep_start_pct = 0.0
        self.sweep_end_pct = 0.0
        self.sweep_duration_s = 1.0

        self.square_active = False
        self.square_start_time = 0.0
        self.square_min_pct = 15.0
        self.square_max_pct = 35.0
        self.square_period_s = 4.0
        self.square_snapshot_next_index = 0
        self.square_snapshot_patterns: deque[tuple[list[float], list[float]]] = deque(maxlen=12)
        self.square_snapshot_dirty = True
        self.snapshot_window: Optional[tk.Toplevel] = None
        self.snapshot_figure: Optional[Figure] = None
        self.snapshot_axes = None
        self.snapshot_throttle_axes = None
        self.snapshot_canvas: Optional[FigureCanvasTkAgg] = None

        self.settings_window: Optional[tk.Toplevel] = None
        self.telemetry_a_ms_var = tk.StringVar(value="50")
        self.telemetry_b_ms_var = tk.StringVar(value="100")
        self.telemetry_c_ms_var = tk.StringVar(value="250")
        self.rate_status = tk.StringVar(value="Telemetry rate uses controller defaults until applied")

        self.cal_sweep_active = False
        self.cal_sweep_start_time = 0.0
        self.cal_sweep_start_pct = 0.0
        self.cal_sweep_end_pct = 0.0
        self.cal_sweep_duration_s = 1.0
        self.calibration_window: Optional[tk.Toplevel] = None
        self.calibration_figure: Optional[Figure] = None
        self.calibration_axes = None
        self.calibration_canvas: Optional[FigureCanvasTkAgg] = None
        self.calibration_dirty = True
        self.calibration_samples: list[dict[str, object]] = []
        self.cal_start_var = tk.StringVar(value="0")
        self.cal_end_var = tk.StringVar(value="100")
        self.cal_duration_var = tk.StringVar(value="20")
        self.calibration_status = tk.StringVar(value="Feedforward calibration sweep inactive")

        # Automated feedforward RPM endpoint measurement. The throttle opening
        # is held at the existing 0% or 100% command for a fixed ten-second
        # measurement window, then the measured RPM average is written into the
        # corresponding feedforward RPM field.
        self.endpoint_average_active = False
        self.endpoint_average_target_pct = 0.0
        self.endpoint_average_label = ""
        self.endpoint_average_started = 0.0
        self.endpoint_average_duration_s = 10.0
        self.endpoint_average_samples: list[float] = []
        self.endpoint_average_status = tk.StringVar(value="Endpoint RPM averaging idle")

        self.history_seconds = tk.DoubleVar(value=45.0)
        self.rpm_times: deque[float] = deque()
        self.rpm_values: deque[float] = deque()
        self.rpm_monotonic_times: deque[float] = deque()
        self.rpm_monotonic_values: deque[float] = deque()
        self.csv_rows: list[dict[str, object]] = []
        self.session_t0 = time.monotonic()

        self._build_ui()
        self.refresh_ports()
        self._apply_saved_settings()
        self.after(50, self.process_serial_lines)
        self.after(100, self.command_heartbeat_loop)
        self.after(200, self.graph_update_loop)
        self.after(250, self.status_update_loop)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- UI ----------------
    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=12)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=0)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        left = ttk.Frame(root)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 12))
        right = ttk.Frame(root)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self._build_connection_box(left)
        self._build_command_box(left)
        self._build_tuning_box(left)
        self._build_sweep_box(left)
        self._build_square_wave_box(left)
        self._build_log_box(left)
        self._build_telemetry_box(right)
        self._build_graph_box(right)
        self._build_console_box(right)

    def _build_connection_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Bridge connection", padding=10)
        box.pack(fill="x", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Serial port").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(box, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(4, 0))

        self.connect_button = ttk.Button(box, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))

        self.connection_status = tk.StringVar(value="Disconnected")
        ttk.Label(box, textvariable=self.connection_status).grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Button(box, text="Settings / CAN telemetry rate", command=self.open_settings_window).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8, 0))

    def _build_command_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Live command", padding=10)
        box.pack(fill="x", pady=(0, 10))
        box.columnconfigure(0, weight=1)
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Manual throttle (%)").grid(row=0, column=0, columnspan=2, sticky="w")
        self.throttle_slider = ttk.Scale(box, from_=0.0, to=100.0, variable=self.manual_throttle, command=self._slider_changed)
        self.throttle_slider.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self.throttle_text = tk.StringVar(value="0.00 %")
        ttk.Label(box, textvariable=self.throttle_text, font=("TkDefaultFont", 11, "bold")).grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 8))

        ttk.Button(box, text="ARM", command=self.arm).grid(row=3, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(box, text="DISARM", command=self.disarm).grid(row=3, column=1, sticky="ew", padx=(4, 0))
        ttk.Button(box, text="PANIC: ZERO + DISARM", command=self.panic_disarm).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        self.command_status = tk.StringVar(value="Command heartbeat idle")
        ttk.Label(box, textvariable=self.command_status, wraplength=330).grid(row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))

    def _build_tuning_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="PID + feedforward tuning", padding=10)
        box.pack(fill="x", pady=(0, 10))
        for col in range(4):
            box.columnconfigure(col, weight=1)

        self.kp_var = tk.StringVar(value="0.035")
        self.ki_var = tk.StringVar(value="0.012")
        self.kd_var = tk.StringVar(value="0.000")
        self.limit_var = tk.StringVar(value="175")
        self.ff0_rpm_var = tk.StringVar(value="2200")
        self.ff0_us_var = tk.StringVar(value="1850")
        self.ff100_rpm_var = tk.StringVar(value="4250")
        self.ff100_us_var = tk.StringVar(value="1450")

        fields = [
            ("Kp", self.kp_var), ("Ki", self.ki_var),
            ("Kd", self.kd_var), ("PID limit us", self.limit_var),
            ("0% RPM", self.ff0_rpm_var), ("0% us", self.ff0_us_var),
            ("100% RPM", self.ff100_rpm_var), ("100% us", self.ff100_us_var),
        ]
        for i, (label, var) in enumerate(fields):
            r = (i // 2) * 2
            c = (i % 2) * 2
            ttk.Label(box, text=label).grid(row=r, column=c, sticky="w", padx=(0, 5), pady=(0, 2))
            ttk.Entry(box, textvariable=var, width=12).grid(row=r + 1, column=c, columnspan=2, sticky="ew", padx=(0, 7), pady=(0, 6))

        ttk.Button(box, text="Apply all tuning values", command=self.apply_tuning).grid(row=8, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Button(
            box,
            text="Auto-set 0% RPM: command idle opening + average 10 s",
            command=lambda: self.start_endpoint_average("0%"),
        ).grid(row=9, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        ttk.Button(
            box,
            text="Auto-set 100% RPM: command full opening + average 10 s",
            command=lambda: self.start_endpoint_average("100%"),
        ).grid(row=10, column=0, columnspan=4, sticky="ew", pady=(6, 0))
        ttk.Button(
            box,
            text="Cancel endpoint average → 0% throttle",
            command=lambda: self.cancel_endpoint_average(zero_throttle=True),
        ).grid(row=11, column=0, columnspan=4, sticky="ew", pady=(6, 0))
        ttk.Label(box, textvariable=self.endpoint_average_status, wraplength=330).grid(row=12, column=0, columnspan=4, sticky="w", pady=(8, 0))
        self.tuning_status = tk.StringVar(value="No tuning packet sent yet")
        ttk.Label(box, textvariable=self.tuning_status, wraplength=330).grid(row=13, column=0, columnspan=4, sticky="w", pady=(8, 0))

    def _build_sweep_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Throttle sweep", padding=10)
        box.pack(fill="x", pady=(0, 10))
        for col in range(4):
            box.columnconfigure(col, weight=1)

        self.sweep_start_var = tk.StringVar(value="0")
        self.sweep_end_var = tk.StringVar(value="100")
        self.sweep_duration_var = tk.StringVar(value="12")

        ttk.Label(box, text="Start %").grid(row=0, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.sweep_start_var, width=8).grid(row=1, column=0, sticky="ew", padx=(0, 6))
        ttk.Label(box, text="End %").grid(row=0, column=1, sticky="w")
        ttk.Entry(box, textvariable=self.sweep_end_var, width=8).grid(row=1, column=1, sticky="ew", padx=(0, 6))
        ttk.Label(box, text="Duration s").grid(row=0, column=2, sticky="w")
        ttk.Entry(box, textvariable=self.sweep_duration_var, width=8).grid(row=1, column=2, sticky="ew", padx=(0, 6))

        ttk.Button(box, text="Start sweep", command=self.start_sweep).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 4))
        ttk.Button(box, text="Stop → 0%", command=self.stop_sweep_to_zero).grid(row=2, column=2, columnspan=2, sticky="ew", pady=(8, 0), padx=(4, 0))
        self.sweep_status = tk.StringVar(value="Sweep inactive")
        ttk.Label(box, textvariable=self.sweep_status, wraplength=330).grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Button(box, text="Open feedforward calibration sweep", command=self.open_calibration_window).grid(row=4, column=0, columnspan=4, sticky="ew", pady=(8, 0))

    def _build_square_wave_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Square-wave PID tuning", padding=10)
        box.pack(fill="x", pady=(0, 10))
        for col in range(4):
            box.columnconfigure(col, weight=1)

        self.square_min_var = tk.StringVar(value="15")
        self.square_max_var = tk.StringVar(value="35")
        self.square_period_var = tk.StringVar(value="4")

        ttk.Label(box, text="Minimum %").grid(row=0, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.square_min_var, width=8).grid(row=1, column=0, sticky="ew", padx=(0, 6))
        ttk.Label(box, text="Maximum %").grid(row=0, column=1, sticky="w")
        ttk.Entry(box, textvariable=self.square_max_var, width=8).grid(row=1, column=1, sticky="ew", padx=(0, 6))
        ttk.Label(box, text="Period s").grid(row=0, column=2, sticky="w")
        ttk.Entry(box, textvariable=self.square_period_var, width=8).grid(row=1, column=2, sticky="ew", padx=(0, 6))

        ttk.Button(box, text="Start square wave", command=self.start_square_wave).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0), padx=(0, 4))
        ttk.Button(box, text="Stop → 0%", command=self.stop_square_wave_to_zero).grid(row=2, column=2, columnspan=2, sticky="ew", pady=(8, 0), padx=(4, 0))
        ttk.Button(box, text="Open period snapshots", command=self.open_snapshot_window).grid(row=3, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        self.square_status = tk.StringVar(value="Square wave inactive")
        ttk.Label(box, textvariable=self.square_status, wraplength=330).grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 0))

    def _build_log_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Graph + logging", padding=10)
        box.pack(fill="x")
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="Graph window (s)").grid(row=0, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.history_seconds, width=10).grid(row=0, column=1, sticky="ew", padx=(6, 0))
        ttk.Button(box, text="Clear graph", command=self.clear_graph).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        ttk.Button(box, text="Save CSV telemetry", command=self.save_csv).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _build_telemetry_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Live telemetry", padding=10)
        box.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        for col in range(6):
            box.columnconfigure(col, weight=1)

        self.value_vars = {
            "rpm": tk.StringVar(value="0.0 RPM"),
            "state": tk.StringVar(value="DISARMED"),
            "link": tk.StringVar(value="No telemetry"),
            "out": tk.StringVar(value="0 us"),
            "target": tk.StringVar(value="0.0 RPM"),
            "ff": tk.StringVar(value="0 us"),
            "pid": tk.StringVar(value="0.0 us"),
            "temp": tk.StringVar(value="0.0 °C"),
            "current": tk.StringVar(value="0.000 A"),
            "vbus": tk.StringVar(value="0.000 V"),
            "runtime": tk.StringVar(value="0 s"),
            "flags": tk.StringVar(value="flags=0"),
        }
        items = [
            ("RPM", "rpm"), ("State", "state"), ("Link", "link"),
            ("Throttle PWM", "out"), ("Target", "target"), ("Feedforward", "ff"),
            ("PID correction", "pid"), ("Temperature", "temp"), ("Current", "current"),
            ("Bus voltage", "vbus"), ("Runtime", "runtime"), ("Flags", "flags"),
        ]
        for i, (label, key) in enumerate(items):
            r = (i // 6) * 2
            c = i % 6
            ttk.Label(box, text=label).grid(row=r, column=c, sticky="w", padx=6)
            ttk.Label(box, textvariable=self.value_vars[key], font=("TkDefaultFont", 10, "bold")).grid(row=r + 1, column=c, sticky="w", padx=6, pady=(0, 8))

    def _build_graph_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="RPM vs time", padding=10)
        box.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        box.rowconfigure(0, weight=1)
        box.columnconfigure(0, weight=1)

        self.figure = Figure(figsize=(8, 4), dpi=100)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel("RPM")
        self.axes.grid(True)
        (self.rpm_line,) = self.axes.plot([], [])
        self.canvas = FigureCanvasTkAgg(self.figure, master=box)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _build_console_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Bridge console", padding=10)
        box.grid(row=2, column=0, sticky="nsew")
        box.rowconfigure(0, weight=1)
        box.columnconfigure(0, weight=1)
        self.console = tk.Text(box, height=8, wrap="none")
        self.console.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(box, orient="vertical", command=self.console.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.console.configure(yscrollcommand=scroll.set)

    # ---------------- Serial / protocol ----------------
    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports:
            self.port_var.set("")

    def toggle_connection(self) -> None:
        if self.serial_worker.is_connected():
            self._safe_zero_disarm_before_disconnect()
            self.serial_worker.disconnect()
            self.connect_button.configure(text="Connect")
            self.connection_status.set("Disconnected")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Choose a serial port first.")
            return
        try:
            self.serial_worker.connect(port)
            self.connect_button.configure(text="Disconnect")
            self.connection_status.set(f"Connected to {port}")
            self.send_line("PING")
            self.send_line("INFO")
            self.apply_telemetry_rate_settings(show_errors=False)
            self.save_settings(silent=True)
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.connection_status.set("Disconnected")

    def send_line(self, line: str) -> None:
        try:
            self.serial_worker.write_line(line)
        except Exception as exc:
            self.append_console(f"GUI ERR write failed: {exc}")

    def process_serial_lines(self) -> None:
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if line.startswith("__SERIAL_ERROR__"):
                self.append_console(line)
                self.connection_status.set("Serial error / disconnected")
                self.serial_worker.disconnect()
                self.connect_button.configure(text="Connect")
                continue
            self.append_console(line)
            self.parse_bridge_line(line)
        self.after(50, self.process_serial_lines)

    def parse_bridge_line(self, line: str) -> None:
        if line.startswith("TEL A "):
            kv = self.parse_kv(line[6:])
            self.telemetry.rpm = self.to_float(kv.get("rpm"), self.telemetry.rpm)
            self.telemetry.out_us = self.to_int(kv.get("out_us"), self.telemetry.out_us)
            self.telemetry.state = self.to_int(kv.get("state"), self.telemetry.state)
            self.telemetry.flags = self.to_int(kv.get("flags"), self.telemetry.flags)
            self.telemetry.last_rx_monotonic = time.monotonic()
            self.record_rpm_sample()
        elif line.startswith("TEL B "):
            kv = self.parse_kv(line[6:])
            self.telemetry.target_rpm = self.to_float(kv.get("target_rpm"), self.telemetry.target_rpm)
            self.telemetry.ff_us = self.to_int(kv.get("ff_us"), self.telemetry.ff_us)
            self.telemetry.pid_us = self.to_float(kv.get("pid_us"), self.telemetry.pid_us)
            self.telemetry.last_rx_monotonic = time.monotonic()
        elif line.startswith("TEL C "):
            kv = self.parse_kv(line[6:])
            self.telemetry.temp_c = self.to_float(kv.get("temp_c"), self.telemetry.temp_c)
            self.telemetry.current_a = self.to_float(kv.get("current_a"), self.telemetry.current_a)
            self.telemetry.vbus_v = self.to_float(kv.get("vbus_v"), self.telemetry.vbus_v)
            self.telemetry.runtime_s = self.to_int(kv.get("runtime_s"), self.telemetry.runtime_s)
            self.telemetry.last_rx_monotonic = time.monotonic()
        elif line.startswith("OK PID"):
            self.tuning_status.set("Bridge accepted PID CAN frames")
        elif line.startswith("OK FF"):
            self.tuning_status.set("Bridge accepted feedforward CAN frame")
        elif line.startswith("OK RATE"):
            self.rate_status.set("Bridge accepted live telemetry-rate CAN frame")

    @staticmethod
    def parse_kv(text: str) -> dict[str, str]:
        out: dict[str, str] = {}
        for token in text.split():
            if "=" in token:
                k, v = token.split("=", 1)
                out[k] = v
        return out

    @staticmethod
    def to_float(value: Optional[str], fallback: float) -> float:
        try:
            return float(value) if value is not None else fallback
        except ValueError:
            return fallback

    @staticmethod
    def to_int(value: Optional[str], fallback: int) -> int:
        try:
            return int(float(value)) if value is not None else fallback
        except ValueError:
            return fallback

    # ---------------- Commands ----------------
    def _slider_changed(self, _event: object = None) -> None:
        self.throttle_text.set(f"{self.manual_throttle.get():.2f} %")

    def arm(self) -> None:
        self.command_armed = True
        self.command_status.set("ARM command active; heartbeat transmitting while connected")
        self.send_current_command()

    def disarm(self) -> None:
        self.command_armed = False
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.command_status.set("DISARM command active")
        self.sweep_status.set("Sweep inactive")
        self.square_status.set("Square wave inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")
        self.send_current_command()

    def panic_disarm(self) -> None:
        self.disarm()
        self.send_line("CMD 0 0")
        self.command_status.set("PANIC: zero throttle + disarm sent")

    def send_current_command(self) -> None:
        if self.serial_worker.is_connected():
            throttle = self.manual_throttle.get()
            self.send_line(f"CMD {1 if self.command_armed else 0} {throttle:.3f}")

    def command_heartbeat_loop(self) -> None:
        if self.serial_worker.is_connected():
            self.update_active_pattern_value()
            self.send_current_command()
        self.after(100, self.command_heartbeat_loop)

    def start_endpoint_average(self, endpoint_label: str) -> None:
        """Hold one feedforward throttle endpoint and average RPM for ten seconds."""
        if endpoint_label not in {"0%", "100%"}:
            messagebox.showerror("Bad endpoint", "Endpoint must be 0% or 100%.")
            return
        if not self.serial_worker.is_connected():
            messagebox.showerror("Bridge disconnected", "Connect the USB↔CAN bridge before starting endpoint averaging.")
            return

        target_pct = 0.0 if endpoint_label == "0%" else 100.0
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.sweep_status.set("Sweep inactive")
        self.square_status.set("Square wave inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")

        self.endpoint_average_active = True
        self.endpoint_average_target_pct = target_pct
        self.endpoint_average_label = endpoint_label
        self.endpoint_average_started = time.monotonic()
        self.endpoint_average_samples.clear()
        self.manual_throttle.set(target_pct)
        self._slider_changed()
        self.send_current_command()

        arm_note = "" if self.command_armed else " Command is currently DISARMED, so arm/run the controller or the RPM average may stay near zero."
        self.endpoint_average_status.set(
            f"Averaging {endpoint_label} feedforward RPM at {target_pct:.0f}% throttle for 10.0 s.{arm_note}"
        )

    def update_endpoint_average(self) -> None:
        if not self.endpoint_average_active:
            return
        elapsed = max(time.monotonic() - self.endpoint_average_started, 0.0)
        remaining = max(self.endpoint_average_duration_s - elapsed, 0.0)
        self.endpoint_average_status.set(
            f"Averaging {self.endpoint_average_label} feedforward RPM: {remaining:0.1f} s left, {len(self.endpoint_average_samples)} RPM sample(s) captured"
        )
        if elapsed >= self.endpoint_average_duration_s:
            self.finish_endpoint_average()

    def finish_endpoint_average(self) -> None:
        if not self.endpoint_average_active:
            return
        label = self.endpoint_average_label
        samples = list(self.endpoint_average_samples)
        self.endpoint_average_active = False
        self.endpoint_average_target_pct = self.manual_throttle.get()
        self.endpoint_average_label = ""
        self.endpoint_average_started = 0.0
        self.endpoint_average_samples.clear()

        if not samples:
            self.endpoint_average_status.set("Endpoint averaging finished, but no RPM samples were received; feedforward field was not changed.")
            messagebox.showerror("No RPM samples", "No TEL A RPM samples arrived during the 10-second averaging window.")
            return

        avg_rpm = sum(samples) / len(samples)
        if label == "0%":
            self.ff0_rpm_var.set(f"{avg_rpm:.3f}")
        else:
            self.ff100_rpm_var.set(f"{avg_rpm:.3f}")

        self.endpoint_average_status.set(
            f"{label} feedforward RPM auto-set to {avg_rpm:.2f} RPM from {len(samples)} sample(s); applying feedforward update."
        )
        self.apply_tuning()
        self.save_settings(silent=True)

    def cancel_endpoint_average(self, silent: bool = False, zero_throttle: bool = False) -> None:
        was_active = self.endpoint_average_active
        self.endpoint_average_active = False
        self.endpoint_average_label = ""
        self.endpoint_average_started = 0.0
        self.endpoint_average_samples.clear()
        if zero_throttle:
            self.manual_throttle.set(0.0)
            self._slider_changed()
            self.send_current_command()
        if was_active and not silent:
            suffix = "; throttle forced to 0%" if zero_throttle else ""
            self.endpoint_average_status.set(f"Endpoint RPM averaging cancelled{suffix}.")
        elif not was_active and zero_throttle and not silent:
            self.endpoint_average_status.set("No endpoint averaging was active; throttle forced to 0%.")

    def apply_tuning(self) -> None:
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            limit = float(self.limit_var.get())
            ff0_rpm = float(self.ff0_rpm_var.get())
            ff0_us = int(float(self.ff0_us_var.get()))
            ff100_rpm = float(self.ff100_rpm_var.get())
            ff100_us = int(float(self.ff100_us_var.get()))
        except ValueError:
            messagebox.showerror("Bad tuning value", "Every tuning field must be numeric.")
            return

        if min(kp, ki, kd) < 0.0:
            messagebox.showerror("Bad PID", "PID gains must be non-negative.")
            return
        if limit <= 0.0 or limit > 2000.0:
            messagebox.showerror("Bad PID limit", "PID correction limit must be in (0, 2000] us.")
            return
        if ff0_rpm < 0.0 or ff100_rpm <= ff0_rpm:
            messagebox.showerror("Bad RPM endpoints", "100% RPM must be greater than 0% RPM.")
            return
        if not (1000 <= ff0_us <= 2000 and 1000 <= ff100_us <= 2000):
            messagebox.showerror("Bad servo us", "Throttle endpoints must stay inside 1000..2000 us.")
            return
        if ff100_us >= ff0_us:
            messagebox.showerror("Bad throttle direction", "For this engine servo, 100% throttle us must be lower than 0% throttle us.")
            return

        self.send_line(f"PID {kp:.8g} {ki:.8g} {kd:.8g} {limit:.8g}")
        self.send_line(f"FF0 {ff0_rpm:.8g} {ff0_us}")
        self.send_line(f"FF100 {ff100_rpm:.8g} {ff100_us}")
        self.tuning_status.set("PID + feedforward packets sent to bridge")
        self.save_settings(silent=True)

    # ---------------- Settings / persistence ----------------
    @staticmethod
    def _nested_get(data: dict[str, object], path: tuple[str, ...], default: object) -> object:
        cur: object = data
        for key in path:
            if not isinstance(cur, dict) or key not in cur:
                return default
            cur = cur[key]
        return cur

    def _load_settings_file(self) -> dict[str, object]:
        try:
            if not self.settings_path.exists():
                return {}
            with self.settings_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def _apply_saved_settings(self) -> None:
        data = self.saved_settings
        serial_port = self._nested_get(data, ("serial_port",), "")
        if isinstance(serial_port, str) and serial_port:
            known_ports = set(str(p) for p in self.port_combo["values"])
            if serial_port in known_ports:
                self.port_var.set(serial_port)

        def set_string(var: tk.StringVar, path: tuple[str, ...], fallback: str) -> None:
            value = self._nested_get(data, path, fallback)
            var.set(str(value))

        def set_double(var: tk.DoubleVar, path: tuple[str, ...], fallback: float) -> None:
            value = self._nested_get(data, path, fallback)
            try:
                var.set(float(value))
            except (TypeError, ValueError, tk.TclError):
                var.set(fallback)

        set_double(self.history_seconds, ("graph", "history_seconds"), 45.0)
        set_string(self.kp_var, ("pid", "kp"), "0.035")
        set_string(self.ki_var, ("pid", "ki"), "0.012")
        set_string(self.kd_var, ("pid", "kd"), "0.000")
        set_string(self.limit_var, ("pid", "limit_us"), "175")
        set_string(self.ff0_rpm_var, ("feedforward", "rpm_0_pct"), "2200")
        set_string(self.ff0_us_var, ("feedforward", "us_0_pct"), "1850")
        set_string(self.ff100_rpm_var, ("feedforward", "rpm_100_pct"), "4250")
        set_string(self.ff100_us_var, ("feedforward", "us_100_pct"), "1450")
        set_string(self.sweep_start_var, ("sweep", "start_pct"), "0")
        set_string(self.sweep_end_var, ("sweep", "end_pct"), "100")
        set_string(self.sweep_duration_var, ("sweep", "duration_s"), "12")
        set_string(self.square_min_var, ("square_wave", "minimum_pct"), "15")
        set_string(self.square_max_var, ("square_wave", "maximum_pct"), "35")
        set_string(self.square_period_var, ("square_wave", "period_s"), "4")
        set_string(self.telemetry_a_ms_var, ("telemetry_periods_ms", "a"), "50")
        set_string(self.telemetry_b_ms_var, ("telemetry_periods_ms", "b"), "100")
        set_string(self.telemetry_c_ms_var, ("telemetry_periods_ms", "c"), "250")
        set_string(self.cal_start_var, ("calibration_sweep", "start_pct"), "0")
        set_string(self.cal_end_var, ("calibration_sweep", "end_pct"), "100")
        set_string(self.cal_duration_var, ("calibration_sweep", "duration_s"), "20")
        self._slider_changed()

    def _settings_payload(self) -> dict[str, object]:
        return {
            "serial_port": self.port_var.get().strip(),
            "graph": {
                "history_seconds": self.history_seconds.get(),
            },
            "pid": {
                "kp": self.kp_var.get(),
                "ki": self.ki_var.get(),
                "kd": self.kd_var.get(),
                "limit_us": self.limit_var.get(),
            },
            "feedforward": {
                "rpm_0_pct": self.ff0_rpm_var.get(),
                "us_0_pct": self.ff0_us_var.get(),
                "rpm_100_pct": self.ff100_rpm_var.get(),
                "us_100_pct": self.ff100_us_var.get(),
            },
            "telemetry_periods_ms": {
                "a": self.telemetry_a_ms_var.get(),
                "b": self.telemetry_b_ms_var.get(),
                "c": self.telemetry_c_ms_var.get(),
            },
            "sweep": {
                "start_pct": self.sweep_start_var.get(),
                "end_pct": self.sweep_end_var.get(),
                "duration_s": self.sweep_duration_var.get(),
            },
            "square_wave": {
                "minimum_pct": self.square_min_var.get(),
                "maximum_pct": self.square_max_var.get(),
                "period_s": self.square_period_var.get(),
            },
            "calibration_sweep": {
                "start_pct": self.cal_start_var.get(),
                "end_pct": self.cal_end_var.get(),
                "duration_s": self.cal_duration_var.get(),
            },
        }

    def save_settings(self, silent: bool = False) -> None:
        try:
            payload = self._settings_payload()
            with self.settings_path.open("w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, sort_keys=True)
                f.write("\n")
            self.saved_settings = payload
            if not silent:
                messagebox.showinfo("Settings saved", f"Saved GUI settings to:\n{self.settings_path}")
        except (OSError, tk.TclError) as exc:
            if not silent:
                messagebox.showerror("Settings save failed", str(exc))

    def open_settings_window(self) -> None:
        if self.settings_window is not None and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return

        win = tk.Toplevel(self)
        win.title("GUI settings + CAN telemetry sampling")
        win.geometry("620x360")
        win.minsize(560, 320)
        win.protocol("WM_DELETE_WINDOW", self.close_settings_window)
        self.settings_window = win

        frame = ttk.Frame(win, padding=14)
        frame.pack(fill="both", expand=True)
        for col in range(3):
            frame.columnconfigure(col, weight=1)

        ttk.Label(
            frame,
            text="Lower telemetry periods increase CAN/USB serial sampling. TELEM A is RPM/state/output, TELEM B is target/feedforward/PID, TELEM C is slower thermal/current data.",
            wraplength=580,
        ).grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 12))

        fields = [
            ("TELEM A period (ms)", self.telemetry_a_ms_var),
            ("TELEM B period (ms)", self.telemetry_b_ms_var),
            ("TELEM C period (ms)", self.telemetry_c_ms_var),
        ]
        for col, (label, var) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=1, column=col, sticky="w", padx=(0, 8))
            ttk.Entry(frame, textvariable=var, width=14).grid(row=2, column=col, sticky="ew", padx=(0, 8), pady=(4, 0))

        ttk.Button(frame, text="Apply live CAN telemetry rate", command=self.apply_telemetry_rate_settings).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(14, 0))
        ttk.Button(frame, text="Save all GUI settings", command=self.save_settings).grid(row=4, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Label(frame, textvariable=self.rate_status, wraplength=580).grid(row=5, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        ttk.Label(
            frame,
            text=f"Motor firmware accepts {TELEMETRY_PERIOD_MIN_MS}..{TELEMETRY_PERIOD_MAX_MS} ms. Example fast tuning rate: A=10, B=20, C=200.",
            wraplength=580,
        ).grid(row=6, column=0, columnspan=3, sticky="ew", pady=(12, 0))

    def close_settings_window(self) -> None:
        if self.settings_window is not None and self.settings_window.winfo_exists():
            self.settings_window.destroy()
        self.settings_window = None

    def apply_telemetry_rate_settings(self, show_errors: bool = True) -> None:
        try:
            a_ms = int(float(self.telemetry_a_ms_var.get()))
            b_ms = int(float(self.telemetry_b_ms_var.get()))
            c_ms = int(float(self.telemetry_c_ms_var.get()))
        except ValueError:
            if show_errors:
                messagebox.showerror("Bad telemetry periods", "Telemetry A/B/C periods must be numeric milliseconds.")
            return

        if not all(TELEMETRY_PERIOD_MIN_MS <= v <= TELEMETRY_PERIOD_MAX_MS for v in (a_ms, b_ms, c_ms)):
            if show_errors:
                messagebox.showerror(
                    "Bad telemetry periods",
                    f"Each telemetry period must be between {TELEMETRY_PERIOD_MIN_MS} and {TELEMETRY_PERIOD_MAX_MS} ms.",
                )
            return

        self.telemetry_a_ms_var.set(str(a_ms))
        self.telemetry_b_ms_var.set(str(b_ms))
        self.telemetry_c_ms_var.set(str(c_ms))
        if self.serial_worker.is_connected():
            self.send_line(f"RATE {a_ms} {b_ms} {c_ms}")
            self.rate_status.set(f"Telemetry rate sent: A={a_ms} ms, B={b_ms} ms, C={c_ms} ms")
        else:
            self.rate_status.set(f"Telemetry rate saved locally: A={a_ms} ms, B={b_ms} ms, C={c_ms} ms; it will send on connect")
        self.save_settings(silent=True)

    # ---------------- Feedforward calibration sweep window ----------------
    def open_calibration_window(self) -> None:
        if self.calibration_window is not None and self.calibration_window.winfo_exists():
            self.calibration_window.lift()
            self.update_calibration_plot()
            return

        win = tk.Toplevel(self)
        win.title("Feedforward calibration sweep")
        win.geometry("980x700")
        win.minsize(820, 600)
        win.protocol("WM_DELETE_WINDOW", self.close_calibration_window)
        self.calibration_window = win

        frame = ttk.Frame(win, padding=12)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(3, weight=1)

        controls = ttk.LabelFrame(frame, text="Commanded throttle sweep", padding=10)
        controls.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        for col in range(6):
            controls.columnconfigure(col, weight=1)

        ttk.Label(controls, text="Start %").grid(row=0, column=0, sticky="w")
        ttk.Entry(controls, textvariable=self.cal_start_var, width=10).grid(row=1, column=0, sticky="ew", padx=(0, 6))
        ttk.Label(controls, text="End %").grid(row=0, column=1, sticky="w")
        ttk.Entry(controls, textvariable=self.cal_end_var, width=10).grid(row=1, column=1, sticky="ew", padx=(0, 6))
        ttk.Label(controls, text="Duration s").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.cal_duration_var, width=10).grid(row=1, column=2, sticky="ew", padx=(0, 6))
        ttk.Button(controls, text="Start calibration sweep", command=self.start_calibration_sweep).grid(row=1, column=3, sticky="ew", padx=(0, 6))
        ttk.Button(controls, text="Stop → 0%", command=self.stop_calibration_sweep_to_zero).grid(row=1, column=4, sticky="ew", padx=(0, 6))
        ttk.Button(controls, text="Clear captured data", command=self.clear_calibration_samples).grid(row=1, column=5, sticky="ew")

        ttk.Label(frame, textvariable=self.calibration_status, wraplength=930).grid(row=1, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(frame, text="Save calibration CSV", command=self.save_calibration_csv).grid(row=2, column=0, sticky="ew", pady=(0, 10))

        self.calibration_figure = Figure(figsize=(8.8, 4.8), dpi=100)
        self.calibration_axes = self.calibration_figure.add_subplot(111)
        self.calibration_canvas = FigureCanvasTkAgg(self.calibration_figure, master=frame)
        self.calibration_canvas.get_tk_widget().grid(row=3, column=0, sticky="nsew")
        self.calibration_dirty = True
        self.update_calibration_plot()

    def close_calibration_window(self) -> None:
        if self.calibration_window is not None and self.calibration_window.winfo_exists():
            self.calibration_window.destroy()
        self.calibration_window = None
        self.calibration_figure = None
        self.calibration_axes = None
        self.calibration_canvas = None

    def start_calibration_sweep(self) -> None:
        try:
            start = float(self.cal_start_var.get())
            end = float(self.cal_end_var.get())
            duration = float(self.cal_duration_var.get())
        except ValueError:
            messagebox.showerror("Bad calibration sweep", "Calibration start, end, and duration must be numeric.")
            return
        if not (0.0 <= start <= 100.0 and 0.0 <= end <= 100.0):
            messagebox.showerror("Bad calibration sweep", "Calibration endpoints must be between 0 and 100 percent.")
            return
        if duration <= 0.05:
            messagebox.showerror("Bad calibration sweep", "Calibration duration must be greater than 0.05 seconds.")
            return

        self.sweep_active = False
        self.square_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.sweep_status.set("Sweep inactive")
        self.square_status.set("Square wave inactive")
        self.cal_sweep_active = True
        self.cal_sweep_start_time = time.monotonic()
        self.cal_sweep_start_pct = start
        self.cal_sweep_end_pct = end
        self.cal_sweep_duration_s = duration
        self.calibration_samples.clear()
        self.calibration_dirty = True
        self.manual_throttle.set(start)
        self._slider_changed()
        self.calibration_status.set(
            f"Calibration sweep active: {start:.2f}% → {end:.2f}% over {duration:.2f} s; RPM-vs-throttle samples are being captured"
        )
        self.save_settings(silent=True)
        self.send_current_command()
        self.update_calibration_plot()

    def update_calibration_sweep_value(self) -> None:
        if not self.cal_sweep_active:
            return
        elapsed = time.monotonic() - self.cal_sweep_start_time
        alpha = min(max(elapsed / self.cal_sweep_duration_s, 0.0), 1.0)
        pct = self.cal_sweep_start_pct + alpha * (self.cal_sweep_end_pct - self.cal_sweep_start_pct)
        self.manual_throttle.set(pct)
        self._slider_changed()
        if alpha >= 1.0:
            self.cal_sweep_active = False
            self.calibration_status.set(
                f"Calibration sweep complete — holding {self.cal_sweep_end_pct:.2f}%; captured {len(self.calibration_samples)} sample(s)"
            )

    def stop_calibration_sweep_to_zero(self) -> None:
        self.cal_sweep_active = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.calibration_status.set(
            f"Calibration sweep stopped; throttle forced to 0%; retained {len(self.calibration_samples)} sample(s)"
        )
        self.send_current_command()

    def record_calibration_sample(self, sample_time_s: float) -> None:
        if not self.cal_sweep_active:
            return
        self.calibration_samples.append({
            "time_s": f"{sample_time_s:.6f}",
            "command_throttle_pct": f"{self.manual_throttle.get():.6f}",
            "rpm": f"{self.telemetry.rpm:.6f}",
            "out_us": self.telemetry.out_us,
            "state": self.telemetry.state,
            "target_rpm": f"{self.telemetry.target_rpm:.6f}",
            "ff_us": self.telemetry.ff_us,
            "pid_us": f"{self.telemetry.pid_us:.6f}",
        })
        self.calibration_dirty = True

    def clear_calibration_samples(self) -> None:
        self.calibration_samples.clear()
        self.calibration_dirty = True
        self.calibration_status.set("Calibration capture cleared")
        self.update_calibration_plot()

    def update_calibration_plot(self) -> None:
        if (
            self.calibration_window is None
            or not self.calibration_window.winfo_exists()
            or self.calibration_axes is None
            or self.calibration_canvas is None
        ):
            return

        self.calibration_axes.clear()
        self.calibration_axes.set_title("Feedforward calibration: measured RPM vs commanded throttle")
        self.calibration_axes.set_xlabel("Commanded throttle (%)")
        self.calibration_axes.set_ylabel("Measured RPM")
        self.calibration_axes.grid(True)
        if self.calibration_samples:
            xs = [float(row["command_throttle_pct"]) for row in self.calibration_samples]
            ys = [float(row["rpm"]) for row in self.calibration_samples]
            self.calibration_axes.plot(xs, ys, marker=".", linewidth=1.2)
        else:
            self.calibration_axes.text(
                0.5,
                0.5,
                "No calibration samples captured yet",
                transform=self.calibration_axes.transAxes,
                ha="center",
                va="center",
            )
        self.calibration_figure.tight_layout()
        self.calibration_canvas.draw_idle()
        self.calibration_dirty = False

    def save_calibration_csv(self) -> None:
        if not self.calibration_samples:
            messagebox.showinfo("No calibration data", "No feedforward calibration sweep samples have been captured yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Save feedforward calibration CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile="feedforward_calibration_sweep.csv",
        )
        if not path:
            return
        keys = list(self.calibration_samples[0].keys())
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.calibration_samples)
        messagebox.showinfo("Saved", f"Saved {len(self.calibration_samples)} calibration samples to:\n{path}")

    # ---------------- Sweep ----------------
    def start_sweep(self) -> None:
        try:
            start = float(self.sweep_start_var.get())
            end = float(self.sweep_end_var.get())
            duration = float(self.sweep_duration_var.get())
        except ValueError:
            messagebox.showerror("Bad sweep", "Sweep start, end, and duration must be numeric.")
            return
        if not (0.0 <= start <= 100.0 and 0.0 <= end <= 100.0):
            messagebox.showerror("Bad sweep", "Sweep endpoints must be between 0 and 100 percent.")
            return
        if duration <= 0.05:
            messagebox.showerror("Bad sweep", "Sweep duration must be greater than 0.05 seconds.")
            return
        self.square_active = False
        self.cal_sweep_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.square_status.set("Square wave inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")
        self.sweep_active = True
        self.sweep_start_time = time.monotonic()
        self.sweep_start_pct = start
        self.sweep_end_pct = end
        self.sweep_duration_s = duration
        self.manual_throttle.set(start)
        self._slider_changed()
        self.sweep_status.set(f"Sweeping {start:.2f}% → {end:.2f}% over {duration:.2f} s")
        self.send_current_command()

    def update_active_pattern_value(self) -> None:
        if self.endpoint_average_active:
            self.update_endpoint_average()
        elif self.square_active:
            self.update_square_wave_value()
        elif self.cal_sweep_active:
            self.update_calibration_sweep_value()
        elif self.sweep_active:
            self.update_sweep_value()

    def update_sweep_value(self) -> None:
        if not self.sweep_active:
            return
        elapsed = time.monotonic() - self.sweep_start_time
        alpha = min(max(elapsed / self.sweep_duration_s, 0.0), 1.0)
        pct = self.sweep_start_pct + alpha * (self.sweep_end_pct - self.sweep_start_pct)
        self.manual_throttle.set(pct)
        self._slider_changed()
        if alpha >= 1.0:
            self.sweep_active = False
            self.sweep_status.set(f"Sweep complete — holding {self.sweep_end_pct:.2f}%")

    def stop_sweep_to_zero(self) -> None:
        self.sweep_active = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.sweep_status.set("Sweep stopped; throttle forced to 0%")
        self.send_current_command()

    # ---------------- Square-wave PID tuning ----------------
    def start_square_wave(self) -> None:
        try:
            min_pct = float(self.square_min_var.get())
            max_pct = float(self.square_max_var.get())
            period_s = float(self.square_period_var.get())
        except ValueError:
            messagebox.showerror("Bad square wave", "Minimum, maximum, and period must be numeric.")
            return

        if not (0.0 <= min_pct <= 100.0 and 0.0 <= max_pct <= 100.0):
            messagebox.showerror("Bad square wave", "Square-wave throttle endpoints must be between 0 and 100 percent.")
            return
        if max_pct <= min_pct:
            messagebox.showerror("Bad square wave", "Maximum throttle must be greater than minimum throttle.")
            return
        if period_s <= 0.20:
            messagebox.showerror("Bad square wave", "Period must be greater than 0.20 seconds.")
            return

        self.sweep_active = False
        self.cal_sweep_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.sweep_status.set("Sweep inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")
        self.square_active = True
        self.square_start_time = time.monotonic()
        self.square_min_pct = min_pct
        self.square_max_pct = max_pct
        self.square_period_s = period_s
        self.square_snapshot_next_index = 0
        self.square_snapshot_patterns.clear()
        self.square_snapshot_dirty = True
        self.manual_throttle.set(min_pct)
        self._slider_changed()
        self.square_status.set(
            f"Square wave active: {min_pct:.2f}% ↔ {max_pct:.2f}%, period {period_s:.3f} s; pulse centered in each captured period"
        )
        self.send_current_command()
        self.update_snapshot_plot()

    def update_square_wave_value(self) -> None:
        if not self.square_active:
            return
        elapsed = max(time.monotonic() - self.square_start_time, 0.0)
        phase = elapsed % self.square_period_s
        high_start = 0.25 * self.square_period_s
        high_end = 0.75 * self.square_period_s
        pct = self.square_max_pct if high_start <= phase < high_end else self.square_min_pct
        self.manual_throttle.set(pct)
        self._slider_changed()

    def stop_square_wave_to_zero(self) -> None:
        self.square_active = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.square_status.set("Square wave stopped; throttle forced to 0%")
        self.send_current_command()

    def capture_completed_square_periods(self) -> None:
        if not self.square_active:
            return
        if self.square_start_time <= 0.0 or self.square_period_s <= 0.0:
            return

        now = time.monotonic()
        while True:
            period_start = self.square_start_time + self.square_snapshot_next_index * self.square_period_s
            period_end = period_start + self.square_period_s
            if now < period_end:
                break

            center = period_start + 0.5 * self.square_period_s
            xs: list[float] = []
            ys: list[float] = []
            for sample_t, sample_rpm in zip(self.rpm_monotonic_times, self.rpm_monotonic_values):
                if period_start <= sample_t <= period_end:
                    xs.append(sample_t - center)
                    ys.append(sample_rpm)
            if len(xs) >= 2:
                self.square_snapshot_patterns.append((xs, ys))
                self.square_snapshot_dirty = True
                self.square_status.set(
                    f"Square wave active: captured {len(self.square_snapshot_patterns)} recent period snapshot(s)"
                    if self.square_active else
                    f"Square wave stopped: captured {len(self.square_snapshot_patterns)} recent period snapshot(s)"
                )
            self.square_snapshot_next_index += 1

    def open_snapshot_window(self) -> None:
        if self.snapshot_window is not None and self.snapshot_window.winfo_exists():
            self.snapshot_window.lift()
            self.update_snapshot_plot()
            return

        win = tk.Toplevel(self)
        win.title("Square-wave period snapshots")
        win.geometry("980x620")
        win.minsize(760, 480)
        win.protocol("WM_DELETE_WINDOW", self.close_snapshot_window)
        self.snapshot_window = win

        frame = ttk.Frame(win, padding=12)
        frame.pack(fill="both", expand=True)
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)

        note = ttk.Label(
            frame,
            text="Each trace is one completed square-wave period, centered on the high-throttle pulse. Older traces fade; the newest captured response is darkest.",
            wraplength=900,
        )
        note.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        self.snapshot_figure = Figure(figsize=(8.8, 4.9), dpi=100)
        self.snapshot_axes = self.snapshot_figure.add_subplot(111)
        self.snapshot_throttle_axes = self.snapshot_axes.twinx()
        self.snapshot_canvas = FigureCanvasTkAgg(self.snapshot_figure, master=frame)
        self.snapshot_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        self.square_snapshot_dirty = True
        self.update_snapshot_plot()

    def close_snapshot_window(self) -> None:
        if self.snapshot_window is not None and self.snapshot_window.winfo_exists():
            self.snapshot_window.destroy()
        self.snapshot_window = None
        self.snapshot_figure = None
        self.snapshot_axes = None
        self.snapshot_throttle_axes = None
        self.snapshot_canvas = None

    def update_snapshot_plot(self) -> None:
        if (
            self.snapshot_window is None
            or not self.snapshot_window.winfo_exists()
            or self.snapshot_axes is None
            or self.snapshot_throttle_axes is None
            or self.snapshot_canvas is None
        ):
            return

        self.snapshot_axes.clear()
        self.snapshot_throttle_axes.clear()
        self.snapshot_axes.set_title("RPM response aligned to centered throttle pulse")
        self.snapshot_axes.set_xlabel("Time from pulse center (s)")
        self.snapshot_axes.set_ylabel("RPM")
        self.snapshot_axes.grid(True)
        self.snapshot_throttle_axes.set_ylabel("Throttle command (%)")

        period = max(self.square_period_s, 0.20)
        half = 0.5 * period
        quarter = 0.25 * period
        xs_step = [-half, -quarter, -quarter, quarter, quarter, half]
        ys_step = [self.square_min_pct, self.square_min_pct, self.square_max_pct, self.square_max_pct, self.square_min_pct, self.square_min_pct]
        self.snapshot_throttle_axes.plot(xs_step, ys_step, linestyle="--", linewidth=1.4, label="Throttle command")
        low_lim = min(self.square_min_pct, self.square_max_pct) - 5.0
        high_lim = max(self.square_min_pct, self.square_max_pct) + 5.0
        self.snapshot_throttle_axes.set_ylim(max(-1.0, low_lim), min(101.0, high_lim))

        patterns = list(self.square_snapshot_patterns)
        if patterns:
            n = len(patterns)
            for i, (xs, ys) in enumerate(patterns):
                alpha = 0.12 + 0.83 * ((i + 1) / n)
                width = 2.2 if i == n - 1 else 1.2
                label = "Newest RPM response" if i == n - 1 else None
                self.snapshot_axes.plot(xs, ys, alpha=alpha, linewidth=width, label=label)
            self.snapshot_axes.legend(loc="upper left")
        else:
            self.snapshot_axes.text(
                0.5,
                0.5,
                "No completed square-wave period captured yet",
                transform=self.snapshot_axes.transAxes,
                ha="center",
                va="center",
            )

        self.snapshot_axes.set_xlim(-half, half)
        self.snapshot_throttle_axes.legend(loc="upper right")
        self.snapshot_figure.tight_layout()
        self.snapshot_canvas.draw_idle()
        self.square_snapshot_dirty = False

    # ---------------- Telemetry / graph / logging ----------------
    def record_rpm_sample(self) -> None:
        now = time.monotonic()
        t_rel = now - self.session_t0
        self.rpm_times.append(t_rel)
        self.rpm_values.append(self.telemetry.rpm)
        self.rpm_monotonic_times.append(now)
        self.rpm_monotonic_values.append(self.telemetry.rpm)
        if self.endpoint_average_active:
            self.endpoint_average_samples.append(self.telemetry.rpm)

        self.csv_rows.append({
            "time_s": f"{t_rel:.6f}",
            "rpm": f"{self.telemetry.rpm:.6f}",
            "state": self.telemetry.state,
            "out_us": self.telemetry.out_us,
            "target_rpm": f"{self.telemetry.target_rpm:.6f}",
            "ff_us": self.telemetry.ff_us,
            "pid_us": f"{self.telemetry.pid_us:.6f}",
            "temp_c": f"{self.telemetry.temp_c:.6f}",
            "current_a": f"{self.telemetry.current_a:.6f}",
            "vbus_v": f"{self.telemetry.vbus_v:.6f}",
            "runtime_s": self.telemetry.runtime_s,
            "flags": self.telemetry.flags,
            "command_armed": int(self.command_armed),
            "command_throttle_pct": f"{self.manual_throttle.get():.6f}",
        })
        self.record_calibration_sample(t_rel)
        self.trim_history()
        self.trim_snapshot_sample_buffer()
        self.capture_completed_square_periods()

    def trim_history(self) -> None:
        try:
            window = max(float(self.history_seconds.get()), 1.0)
        except (ValueError, tk.TclError):
            window = 45.0
        if not self.rpm_times:
            return
        newest = self.rpm_times[-1]
        while self.rpm_times and (newest - self.rpm_times[0]) > window:
            self.rpm_times.popleft()
            self.rpm_values.popleft()

    def trim_snapshot_sample_buffer(self) -> None:
        # Keep enough raw data to extract several full square-wave periods even if
        # the main graph history window is set very short.
        keep_s = 180.0
        if self.square_period_s > 0.0:
            keep_s = max(keep_s, 4.0 * self.square_period_s)
        cutoff = time.monotonic() - keep_s
        while self.rpm_monotonic_times and self.rpm_monotonic_times[0] < cutoff:
            self.rpm_monotonic_times.popleft()
            self.rpm_monotonic_values.popleft()

    def graph_update_loop(self) -> None:
        self.trim_history()
        self.trim_snapshot_sample_buffer()
        self.capture_completed_square_periods()
        if self.rpm_times:
            xs = list(self.rpm_times)
            ys = list(self.rpm_values)
            self.rpm_line.set_data(xs, ys)
            self.axes.relim()
            self.axes.autoscale_view()
        else:
            self.rpm_line.set_data([], [])
        self.canvas.draw_idle()
        if self.square_snapshot_dirty or self.square_active:
            self.update_snapshot_plot()
        if self.calibration_dirty or self.cal_sweep_active:
            self.update_calibration_plot()
        self.after(200, self.graph_update_loop)

    def status_update_loop(self) -> None:
        t = self.telemetry
        stale_s = time.monotonic() - t.last_rx_monotonic if t.last_rx_monotonic else 1e9
        link_text = "LIVE" if stale_s < 1.0 else f"STALE ({stale_s:.1f}s)"
        state_name = STATE_NAMES.get(t.state, f"UNKNOWN({t.state})")

        self.value_vars["rpm"].set(f"{t.rpm:.1f} RPM")
        self.value_vars["state"].set(state_name)
        self.value_vars["link"].set(link_text)
        self.value_vars["out"].set(f"{t.out_us} us")
        self.value_vars["target"].set(f"{t.target_rpm:.1f} RPM")
        self.value_vars["ff"].set(f"{t.ff_us} us")
        self.value_vars["pid"].set(f"{t.pid_us:.1f} us")
        self.value_vars["temp"].set(f"{t.temp_c:.1f} °C")
        self.value_vars["current"].set(f"{t.current_a:.3f} A")
        self.value_vars["vbus"].set(f"{t.vbus_v:.3f} V")
        self.value_vars["runtime"].set(f"{t.runtime_s} s")
        self.value_vars["flags"].set(
            f"armed={int(t.armed_flag)} link={int(t.command_link_flag)} stationary={int(t.stationary_flag)}"
        )
        self.after(250, self.status_update_loop)

    def clear_graph(self) -> None:
        self.rpm_times.clear()
        self.rpm_values.clear()
        self.rpm_monotonic_times.clear()
        self.rpm_monotonic_values.clear()
        self.square_snapshot_patterns.clear()
        if self.square_active and self.square_period_s > 0.0:
            elapsed = max(time.monotonic() - self.square_start_time, 0.0)
            # Skip the partly observed period around the clear action; the next
            # snapshot should be a clean complete period captured after clearing.
            self.square_snapshot_next_index = int(elapsed // self.square_period_s) + 1
        else:
            self.square_snapshot_next_index = 0
        self.square_snapshot_dirty = True
        self.session_t0 = time.monotonic()
        self.rpm_line.set_data([], [])
        self.canvas.draw_idle()
        self.update_snapshot_plot()

    def save_csv(self) -> None:
        if not self.csv_rows:
            messagebox.showinfo("No data", "No telemetry samples have been collected yet.")
            return
        path = filedialog.asksaveasfilename(
            title="Save telemetry CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile="engine_telemetry.csv",
        )
        if not path:
            return
        keys = list(self.csv_rows[0].keys())
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.csv_rows)
        messagebox.showinfo("Saved", f"Saved {len(self.csv_rows)} samples to:\n{path}")

    def append_console(self, line: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.console.insert("end", f"[{stamp}] {line}\n")
        self.console.see("end")
        # Keep the text widget bounded.
        try:
            line_count = int(self.console.index("end-1c").split(".")[0])
            if line_count > 500:
                self.console.delete("1.0", "100.0")
        except Exception:
            pass

    # ---------------- Shutdown ----------------
    def _safe_zero_disarm_before_disconnect(self) -> None:
        if self.serial_worker.is_connected():
            try:
                self.serial_worker.write_line("CMD 0 0")
            except Exception:
                pass
        self.command_armed = False
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.manual_throttle.set(0.0)
        self._slider_changed()

    def on_close(self) -> None:
        self.save_settings(silent=True)
        self._safe_zero_disarm_before_disconnect()
        self.serial_worker.disconnect()
        self.destroy()


def main() -> None:
    app = EngineGui()
    app.mainloop()


if __name__ == "__main__":
    main()
