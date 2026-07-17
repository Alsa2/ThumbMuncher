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
import warnings
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk, simpledialog

import serial
import numpy as np
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
    7: "HALL_AUTO_CAL",
    8: "MANUAL_PWM_TEST",
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


@dataclass
class BoardInfo:
    board_id: int
    rpm: int = 0
    state: int = 0
    flags: int = 0
    last_rx_monotonic: float = 0.0

    @property
    def selected(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def armed(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def fc_seen(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def debug_seen(self) -> bool:
        return bool(self.flags & 0x08)


@dataclass
class SquarePeriodSnapshot:
    rpm_x_s: list[float]
    rpm_y: list[float]
    command_x_s: list[float]
    command_pct: list[float]
    pwm_x_s: list[float]
    pwm_us: list[float]
    max_command_edge_delay_ms: Optional[float] = None


@dataclass
class FeedforwardCalibrationPoint:
    throttle_pct: float
    measured_rpm: float
    pwm_us: float
    source: str
    use_for_fit: bool = True


@dataclass
class RpmScopeCapture:
    trigger_rpm: float
    width_s: float
    rpm_x_s: list[float]
    rpm_y: list[float]
    command_x_s: list[float]
    command_pct: list[float]
    pwm_x_s: list[float]
    pwm_us: list[float]


class CollapsibleSection(ttk.Frame):
    def __init__(self, parent: tk.Widget, title: str, open_by_default: bool = False) -> None:
        super().__init__(parent)
        self._open = open_by_default
        self._title = title
        self.button = ttk.Button(self, command=self.toggle)
        self.button.pack(fill="x")
        self.body = ttk.Frame(self)
        self._sync()

    def toggle(self) -> None:
        self._open = not self._open
        self._sync()

    def _sync(self) -> None:
        self.button.configure(text=("▾ " if self._open else "▸ ") + self._title)
        if self._open:
            self.body.pack(fill="x", pady=(6, 0))
        else:
            self.body.pack_forget()


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
        self.geometry("1600x860")
        self.minsize(1300, 760)

        self.telemetry = Telemetry()
        self.last_tel_a_monotonic = 0.0
        self.last_selected_board_fallback_monotonic = 0.0
        self.last_selection_repair_monotonic = 0.0
        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.serial_worker = SerialWorker(self.rx_queue)
        self.settings_path = Path(__file__).with_name(SETTINGS_FILE_NAME)
        self.saved_settings = self._load_settings_file()

        # Main-page configuration edit protection. A typed value remains local
        # until the selected controller confirms the same value. Unsolicited or
        # stale GETCFG/AUTO frames are not allowed to replace pending edits.
        self.config_dirty_vars: set[str] = set()
        self.config_pending_values: dict[str, float] = {}
        self.config_pending_boards: dict[str, int] = {}
        self.config_entry_widgets: dict[str, ttk.Entry] = {}
        self.config_pull_overwrite_vars: set[str] = set()
        self.ff_model_pull_requested = False

        self.boards: dict[int, BoardInfo] = {}
        self.selected_board_id: Optional[int] = None
        self.selected_board_var = tk.StringVar(value="")
        self.board_status = tk.StringVar(value="No boards discovered yet")
        self.auto_status = tk.StringVar(value="Endpoint servo auto-adjust idle")
        self.endpoint_auto_rate_var = tk.StringVar(value="25")
        self.endpoint_auto_duration_var = tk.StringVar(value="15")
        self.endpoint_auto_start_us_var = tk.StringVar(value="1700")
        self.all_cal_active = False
        self.all_cal_queue: list[int] = []
        self.all_cal_endpoint_label = ""
        self.all_cal_phase = "idle"
        self.all_cal_phase_deadline = 0.0
        self.selected_autocal_active = False
        self.selected_autocal_board_id: Optional[int] = None
        self.selected_autocal_phase = "idle"
        self.selected_autocal_phase_deadline = 0.0

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
        self.square_snapshot_patterns: deque[SquarePeriodSnapshot] = deque(maxlen=12)
        self.square_snapshot_dirty = True
        self.snapshot_window: Optional[tk.Toplevel] = None
        self.snapshot_figure: Optional[Figure] = None
        self.snapshot_axes = None
        self.snapshot_throttle_axes = None
        self.snapshot_pwm_axes = None
        self.snapshot_canvas: Optional[FigureCanvasTkAgg] = None

        # Single-shot RPM-triggered oscilloscope. The requested width is the
        # total horizontal span, split evenly before and after the rising RPM
        # threshold crossing. The rolling monotonic buffers provide pre-trigger
        # history, then capture completes after the post-trigger half-window.
        self.scope_trigger_rpm_var = tk.StringVar(value="3000")
        self.scope_width_s_var = tk.StringVar(value="4")
        self.scope_status = tk.StringVar(value="RPM scope idle")
        self.scope_armed = False
        self.scope_triggered = False
        self.scope_trigger_time = 0.0
        self.scope_trigger_rpm = 3000.0
        self.scope_width_s = 4.0
        self.scope_capture: Optional[RpmScopeCapture] = None
        self.scope_dirty = True
        self.scope_window: Optional[tk.Toplevel] = None
        self.scope_figure: Optional[Figure] = None
        self.scope_axes = None
        self.scope_throttle_axes = None
        self.scope_pwm_axes = None
        self.scope_canvas: Optional[FigureCanvasTkAgg] = None

        self.settings_window: Optional[tk.Toplevel] = None
        self.telemetry_a_ms_var = tk.StringVar(value="50")
        self.telemetry_b_ms_var = tk.StringVar(value="100")
        self.telemetry_c_ms_var = tk.StringVar(value="250")
        self.rate_status = tk.StringVar(value="Telemetry rate uses controller defaults until applied")

        # Dedicated per-engine feedforward calibration and model fitting.
        # Calibration commands use direct PWM bypass only and never persist a
        # model. Pull and Send/Commit are explicit separate actions.
        self.cal_sweep_active = False  # legacy flag kept for shared stop logic
        self.calibration_window: Optional[tk.Toplevel] = None
        self.calibration_figure: Optional[Figure] = None
        self.calibration_axes = None
        self.calibration_rpm_axes = None
        self.calibration_canvas: Optional[FigureCanvasTkAgg] = None
        self.calibration_dirty = True
        self.calibration_samples: list[dict[str, object]] = []
        self.ff_calibration_points: list[FeedforwardCalibrationPoint] = []
        self.ff_fit_cache: Optional[dict[str, object]] = None
        self.ff_model_receive: dict[str, object] = {"coefficients": {}, "points": {}}
        self.ff_model_pulled_board_id: Optional[int] = None
        self.ff_model_pulled_snapshot: Optional[dict[str, object]] = None
        # Board-confirmed endpoint RPM values keep the main-page quick PWM button
        # from accidentally sending an unsaved/edit-in-progress RPM field.
        self.ff0_rpm_confirmed_by_board: dict[int, float] = {}
        self.ff_pending_commit_snapshot: Optional[dict[str, object]] = None
        self.ff_model_dirty = False
        self.ff_model_type_var = tk.StringVar(value="Piecewise linear")
        self.ff_poly_order_var = tk.StringVar(value="2")
        self.ff_model_point_count_var = tk.StringVar(value="7")
        self.ff_cal_start_us_var = tk.StringVar(value="1700")
        self.ff_cal_rate_var = tk.StringVar(value="25")
        self.ff_cal_deadband_var = tk.StringVar(value="50")
        self.ff_cal_stable_s_var = tk.StringVar(value="2")
        self.ff_intermediate_min_var = tk.StringVar(value="10")
        self.ff_intermediate_max_var = tk.StringVar(value="90")
        self.ff_point_time_var = tk.StringVar(value="5")
        self.ff_point_timeout_var = tk.StringVar(value="45")
        self.ff_sweep_total_s_var = tk.StringVar(value="30")
        # Legacy endpoint fields are still loaded for backwards-compatible settings,
        # but sweep motion now always uses the selected engine's exact pulled 0%
        # and 100% PWM endpoints.
        self.ff_sweep_start_us_var = tk.StringVar(value="1850")
        self.ff_sweep_end_us_var = tk.StringVar(value="1450")
        self.ff_sweep_endpoint_dwell_s_var = tk.StringVar(value="2")
        self.ff_use_sweep_var = tk.BooleanVar(value=True)
        self.calibration_status = tk.StringVar(value="Calibration idle — pull the selected engine before sending a model")
        self.ff_model_summary_var = tk.StringVar(value="No model pulled")
        self.ff_cal_mode = "idle"
        self.ff_cal_board_id: Optional[int] = None
        self.ff_cal_target_pct = 0.0
        self.ff_cal_target_rpm = 0.0
        self.ff_cal_current_us = 1700.0
        self.ff_cal_last_update = 0.0
        self.ff_cal_stable_since: Optional[float] = None
        self.ff_cal_sample_started: Optional[float] = None
        self.ff_cal_recent_rpm: deque[float] = deque(maxlen=200)
        self.ff_cal_recent_us: deque[float] = deque(maxlen=200)
        self.ff_cal_sequence: list[float] = []
        self.ff_cal_sequence_index = 0
        self.ff_cal_point_deadline = 0.0
        self.ff_sweep_started = 0.0
        self.ff_sweep_duration_s = 1.0
        self.ff_sweep_start_us = 1850.0
        self.ff_sweep_end_us = 1450.0
        self.ff_sweep_endpoint_dwell_s = 2.0
        self.ff_sweep_phase = "idle"
        self.ff_sweep_phase_started = 0.0
        self.ff_sweep_command_pct = 0.0
        self.ff_sweep_last_recorded_time = 0.0
        self.ff_sweep_last_recorded_pct: Optional[float] = None
        self.ff_sweep_idle_rpm = 0.0
        self.ff_sweep_max_rpm = 1.0
        self.ff_point_tree: Optional[ttk.Treeview] = None
        self.ff_point_edit_pct_var = tk.StringVar(value="")
        self.ff_point_edit_rpm_var = tk.StringVar(value="")
        self.ff_point_edit_us_var = tk.StringVar(value="")
        self.ff_point_edit_use_var = tk.BooleanVar(value=True)
        self.ff_changed_widgets: list[tk.Widget] = []
        self.ff_model_banner_label: Optional[tk.Label] = None
        self.ff_traces_installed = False

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
        self.hall_cal_target_rpm_var = tk.StringVar(value="2200")
        # Kept for old settings-file compatibility; the GUI now sends HALLCAL
        # without a fixed capture duration, so firmware runs until clean/abort.
        self.hall_cal_duration_var = tk.StringVar(value="0")
        self.hall_auto_cal_active = False
        self.hall_auto_cal_status = tk.StringVar(value="Hall auto-cal idle")

        self.process_popup: Optional[tk.Toplevel] = None
        self.process_popup_status = tk.StringVar(value="")
        self.process_popup_name = ""

        self.history_seconds = tk.DoubleVar(value=45.0)
        self.rpm_times: deque[float] = deque()
        self.rpm_values: deque[float] = deque()
        self.rpm_monotonic_times: deque[float] = deque()
        self.rpm_monotonic_values: deque[float] = deque()
        self.command_monotonic_times: deque[float] = deque()
        self.command_throttle_values: deque[float] = deque()
        self.output_pwm_monotonic_times: deque[float] = deque()
        self.output_pwm_values: deque[float] = deque()
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

        left_container = ttk.Frame(root)
        left_container.grid(row=0, column=0, sticky="nsw", padx=(0, 12))
        left_container.rowconfigure(0, weight=1)
        left_container.columnconfigure(0, weight=1)

        left_canvas = tk.Canvas(left_container, width=700, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=left_canvas.yview)
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        left_canvas.grid(row=0, column=0, sticky="ns")
        left_scrollbar.grid(row=0, column=1, sticky="ns")
        left = ttk.Frame(left_canvas)
        left_window = left_canvas.create_window((0, 0), window=left, anchor="nw")
        left.bind("<Configure>", lambda _e: left_canvas.configure(scrollregion=left_canvas.bbox("all")))
        left_canvas.bind("<Configure>", lambda e: left_canvas.itemconfigure(left_window, width=e.width))

        right = ttk.Frame(root)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self._build_connection_box(left)
        self._build_board_box(left)
        self._build_command_box(left)
        self._build_tuning_box(left)
        advanced = CollapsibleSection(left, "Advanced sweeps, square-wave tests, and log", open_by_default=False)
        advanced.pack(fill="x", pady=(0, 10))
        self._build_sweep_box(advanced.body)
        self._build_square_wave_box(advanced.body)
        self._build_log_box(advanced.body)
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

    def _build_board_box(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Engine board selection", padding=10)
        box.pack(fill="x", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="Board ID").grid(row=0, column=0, sticky="w")
        self.board_combo = ttk.Combobox(box, textvariable=self.selected_board_var, width=22, state="readonly")
        self.board_combo.grid(row=0, column=1, columnspan=2, sticky="ew", padx=6)
        self.board_combo.bind("<<ComboboxSelected>>", lambda _event: self.select_board_from_gui())
        ttk.Button(box, text="Scan", command=self.scan_boards).grid(row=0, column=3, sticky="ew")

        ttk.Button(box, text="Select", command=self.select_board_from_gui).grid(row=1, column=0, sticky="ew", pady=(6, 0), padx=(0, 4))
        ttk.Button(box, text="Beep + flash selected", command=self.identify_selected_board).grid(row=1, column=1, sticky="ew", pady=(6, 0), padx=(0, 4))
        ttk.Button(box, text="Stop blink/beep", command=self.stop_identify_selected_board).grid(row=1, column=2, sticky="ew", pady=(6, 0), padx=(0, 4))
        ttk.Button(box, text="Set ID", command=self.set_selected_board_id).grid(row=1, column=3, sticky="ew", pady=(6, 0))

        ttk.Button(box, text="Open throttle PWM set popup", command=self.open_throttle_pwm_popup).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0), padx=(0, 4))
        ttk.Button(box, text="Panic all engines", command=self.panic_all_boards).grid(row=2, column=2, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Label(box, textvariable=self.board_status, wraplength=330).grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 0))

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
        box = ttk.LabelFrame(parent, text="PID + engine configuration", padding=10)
        box.pack(fill="x", pady=(0, 10))
        for col in range(6):
            box.columnconfigure(col, weight=1)

        self.kp_var = tk.StringVar(value="0.035")
        self.ki_var = tk.StringVar(value="0.012")
        self.kd_var = tk.StringVar(value="0.000")
        self.limit_var = tk.StringVar(value="1000")
        self.ff0_rpm_var = tk.StringVar(value="2200")
        self.ff0_us_var = tk.StringVar(value="1850")
        self.ff100_rpm_var = tk.StringVar(value="4250")
        self.ff100_us_var = tk.StringVar(value="1450")  # hidden compatibility/cache field
        self.start_us_var = tk.StringVar(value="1400")
        self.start_hold_ms_var = tk.StringVar(value="1000")
        self.manual_pwm_hold_ms_var = tk.StringVar(value="10000")
        self.manual_pwm_bypass_var = tk.BooleanVar(value=False)
        self.manual_pwm_slider_us_var = tk.DoubleVar(value=1700.0)
        self.manual_pwm_slider_label = tk.StringVar(value="1700 us")
        self.last_manual_pwm_slider_send = 0.0
        self.last_manual_pwm_bypass_heartbeat = 0.0
        self.hall_high_raw_var = tk.StringVar(value="215")
        self.hall_low_raw_var = tk.StringVar(value="83")

        ttk.Button(box, text="Pull all configs from selected motor", command=self.get_configs_selected_motor).grid(row=0, column=0, columnspan=6, sticky="ew", pady=(0, 8))

        fields = [
            ("Kp", self.kp_var, "kp"), ("Ki", self.ki_var, "ki"),
            ("Kd", self.kd_var, "kd"), ("PID limit us (1000 = full PWM span)", self.limit_var, "limit"),
            ("0% target RPM", self.ff0_rpm_var, "ff0_rpm"), ("0% throttle PWM us (quick adjust)", self.ff0_us_var, "ff0_us"),
            ("100% target RPM", self.ff100_rpm_var, "ff100_rpm"), ("Start us", self.start_us_var, "start_us"),
            ("Start hold ms", self.start_hold_ms_var, "start_hold_ms"),
            ("RPM high raw", self.hall_high_raw_var, "hall_high"), ("RPM low raw", self.hall_low_raw_var, "hall_low"),
        ]
        for i, (label, var, key) in enumerate(fields):
            r = 1 + (i // 2) * 2
            c = 0 if (i % 2) == 0 else 3
            ttk.Label(box, text=label).grid(row=r, column=c, columnspan=2, sticky="w", padx=(0, 5), pady=(0, 2))
            entry = ttk.Entry(box, textvariable=var, width=12)
            entry.grid(row=r + 1, column=c, sticky="ew", padx=(0, 5), pady=(0, 6))
            self.config_entry_widgets[str(var)] = entry
            entry.bind("<KeyRelease>", lambda _e, v=var, lab=label: self._mark_main_config_dirty(v, lab), add="+")
            entry.bind("<<Paste>>", lambda _e, v=var, lab=label: self.after_idle(lambda: self._mark_main_config_dirty(v, lab)), add="+")
            entry.bind("<<Cut>>", lambda _e, v=var, lab=label: self.after_idle(lambda: self._mark_main_config_dirty(v, lab)), add="+")
            ttk.Button(box, text="Send", command=lambda k=key: self.send_config_field(k)).grid(row=r + 1, column=c + 1, columnspan=2, sticky="ew", padx=(0, 7), pady=(0, 6))

        base = 14
        ttk.Button(
            box,
            text="Open feedforward calibration + fitting",
            command=self.open_calibration_window,
        ).grid(row=base, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        ttk.Label(
            box,
            text="Use the quick 0% PWM field above for small idle adjustments. Full endpoint calibration, model selection, sweep data, fitting, pull, and live commit are in the calibration window.",
            wraplength=600,
        ).grid(row=base + 1, column=0, columnspan=6, sticky="w", pady=(5, 0))
        ttk.Button(box, text="Apply PID + startup + RPM thresholds", command=self.apply_tuning).grid(row=base + 2, column=0, columnspan=6, sticky="ew", pady=(8, 0))

        ttk.Label(box, text="Hall auto-cal target RPM (runs until clean)").grid(row=base + 3, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Entry(box, textvariable=self.hall_cal_target_rpm_var, width=8).grid(row=base + 4, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(box, text="Auto-cal Hall min/max at spinner RPM", command=self.start_hall_auto_cal).grid(row=base + 4, column=1, columnspan=5, sticky="ew")
        ttk.Button(box, text="Stop Hall auto-cal", command=self.stop_hall_auto_cal).grid(row=base + 5, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        ttk.Label(box, textvariable=self.hall_auto_cal_status, wraplength=600).grid(row=base + 6, column=0, columnspan=6, sticky="w", pady=(6, 0))

        self.tuning_status = tk.StringVar(value="No configuration packet sent yet")
        ttk.Label(box, textvariable=self.tuning_status, wraplength=600).grid(row=base + 7, column=0, columnspan=6, sticky="w", pady=(8, 0))

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
        ttk.Button(box, text="Open feedforward calibration + fitting", command=self.open_calibration_window).grid(row=4, column=0, columnspan=4, sticky="ew", pady=(8, 0))

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
        ttk.Label(box, textvariable=self.square_status, wraplength=620).grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 0))

        ttk.Separator(box, orient="horizontal").grid(row=5, column=0, columnspan=4, sticky="ew", pady=(10, 8))
        ttk.Label(box, text="RPM scope trigger").grid(row=6, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.scope_trigger_rpm_var, width=10).grid(row=7, column=0, sticky="ew", padx=(0, 6))
        ttk.Label(box, text="Time width / resolution (s)").grid(row=6, column=1, sticky="w")
        ttk.Entry(box, textvariable=self.scope_width_s_var, width=10).grid(row=7, column=1, sticky="ew", padx=(0, 6))
        ttk.Button(box, text="Open + arm RPM oscilloscope", command=self.open_and_arm_rpm_scope).grid(row=7, column=2, columnspan=2, sticky="ew")
        ttk.Label(box, textvariable=self.scope_status, wraplength=620).grid(row=8, column=0, columnspan=4, sticky="w", pady=(8, 0))

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

    # ---------------- Automated-process popup ----------------
    def show_process_popup(self, name: str, detail: str = "") -> None:
        self.process_popup_name = name
        message = f"Process running: {name}"
        if detail:
            message += f"\n{detail}"
        self.process_popup_status.set(message)

        if self.process_popup is not None and self.process_popup.winfo_exists():
            self.process_popup.title(f"Running: {name}")
            self.process_popup.lift()
            return

        win = tk.Toplevel(self)
        win.title(f"Running: {name}")
        win.geometry("520x260")
        win.minsize(460, 220)
        win.transient(self)
        win.attributes("-topmost", True)
        win.protocol("WM_DELETE_WINDOW", self.abort_automated_process)
        self.process_popup = win

        frame = ttk.Frame(win, padding=18)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)

        ttk.Label(
            frame,
            textvariable=self.process_popup_status,
            font=("TkDefaultFont", 13, "bold"),
            wraplength=470,
            justify="center",
        ).grid(row=0, column=0, sticky="nsew", pady=(0, 18))

        abort_button = tk.Button(
            frame,
            text="ABORT PROCESS",
            font=("TkDefaultFont", 18, "bold"),
            height=2,
            command=self.abort_automated_process,
        )
        abort_button.grid(row=1, column=0, sticky="ew", pady=(6, 0))

        ttk.Label(
            frame,
            text="For endpoint auto-adjust, Abort stops tuning and commands 0% throttle without disarming. Other aborts still safe-disarm as needed.",
            wraplength=470,
            justify="center",
        ).grid(row=2, column=0, sticky="ew", pady=(12, 0))

    def update_process_popup(self, detail: str) -> None:
        if self.process_popup is None or not self.process_popup.winfo_exists():
            return
        name = self.process_popup_name or "automated task"
        self.process_popup_status.set(f"Process running: {name}\n{detail}")

    def close_process_popup(self, name: Optional[str] = None) -> None:
        if name is not None and self.process_popup_name and self.process_popup_name != name:
            return
        if self.process_popup is not None and self.process_popup.winfo_exists():
            try:
                self.process_popup.destroy()
            except tk.TclError:
                pass
        self.process_popup = None
        self.process_popup_name = ""
        self.process_popup_status.set("")

    def abort_automated_process(self) -> None:
        name = (self.process_popup_name or "").lower()
        endpoint_abort = "endpoint auto-adjust" in name

        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.all_cal_active = False
        self.all_cal_queue.clear()
        self.all_cal_phase = "idle"
        self.selected_autocal_active = False
        self.selected_autocal_phase = "idle"
        self.hall_auto_cal_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)

        self.manual_throttle.set(0.0)
        self._slider_changed()

        if endpoint_abort:
            # Stop the endpoint auto-tune, but keep the engine armed if it was
            # armed. This is useful during tuning: it returns the servo command
            # to the saved 0% throttle position instead of killing the rail/state.
            if self.serial_worker.is_connected():
                self.send_line("AUTO_STOP")
                self.send_line("STOP_IDENTIFY")
                self.send_current_command()
            self.command_status.set("Endpoint auto-adjust aborted: command is 0% throttle; ARM state preserved")
            self.auto_status.set("Endpoint auto-adjust stopped; command is 0% throttle, not disarmed")
        else:
            self.command_armed = False
            if self.serial_worker.is_connected():
                self.send_line("AUTO_STOP")
                self.send_line("HALLCAL_STOP")
                self.send_line("STOP_IDENTIFY")
                self.send_line("PANIC_ALL")
                self.send_line("CMD 0 0")
            self.command_status.set("ABORT: zero throttle + disarm sent; automated process stopped")
            self.auto_status.set("Endpoint auto-adjust stopped by abort")
            self.hall_auto_cal_status.set("Hall auto-cal stopped by abort")

        self.sweep_status.set("Sweep inactive")
        self.square_status.set("Square wave inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")
        self.endpoint_average_status.set("Endpoint RPM averaging stopped by abort")
        self.close_process_popup()

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
            self.close_process_popup()
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
            self.send_line("PROBE")
            self.apply_telemetry_rate_settings(show_errors=False)
            self.save_settings(silent=True)
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.connection_status.set("Disconnected")

    def send_line(self, line: str) -> bool:
        try:
            self.serial_worker.write_line(line)
            return True
        except Exception as exc:
            self.append_console(f"GUI ERR write failed: {exc}")
            return False

    def process_serial_lines(self) -> None:
        try:
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
                try:
                    self.parse_bridge_line(line)
                except Exception as exc:
                    # Never let one malformed/stale bridge line kill the Tk after()
                    # telemetry pump. The previous edit-guard implementation could
                    # crash here when focus_get() returned a ttk Combobox popdown
                    # path that Tk could not map back to a widget; that made the
                    # graph look frozen/stale right after selecting an engine.
                    self.append_console(f"GUI ERR parse failed for {line!r}: {exc!r}")
                    self.connection_status.set("Connected, but ignored one malformed telemetry line")
        finally:
            self.after(50, self.process_serial_lines)

    def parse_bridge_line(self, line: str) -> None:
        if line.startswith("TEL A "):
            kv = self.parse_kv(line[6:])
            self.telemetry.rpm = self.to_float(kv.get("rpm"), self.telemetry.rpm)
            self.telemetry.out_us = self.to_int(kv.get("out_us"), self.telemetry.out_us)
            self.telemetry.state = self.to_int(kv.get("state"), self.telemetry.state)
            self.telemetry.flags = self.to_int(kv.get("flags"), self.telemetry.flags)
            now = time.monotonic()
            self.telemetry.last_rx_monotonic = now
            self.last_tel_a_monotonic = now
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
            old_runtime_s = self.telemetry.runtime_s
            new_runtime_s = self.to_int(kv.get("runtime_s"), old_runtime_s)
            if old_runtime_s >= 3 and new_runtime_s + 2 < old_runtime_s:
                warning = (
                    f"BOARD RUNTIME RESET DETECTED: selected board uptime changed "
                    f"from {old_runtime_s}s to {new_runtime_s}s. Check USB/CAN console for "
                    "'watchdog_caused_reboot' and power integrity."
                )
                self.append_console("GUI WARN " + warning)
                self.board_status.set(warning)
            self.telemetry.runtime_s = new_runtime_s
            self.telemetry.last_rx_monotonic = time.monotonic()
        elif line.startswith("BOARD "):
            self.parse_board_line(line[6:])
        elif line.startswith("AUTO "):
            kv = self.parse_kv(line[5:])
            endpoint = "100%" if self.to_int(kv.get("endpoint"), 0) == 1 else "0%"
            active = self.to_int(kv.get("active"), 0)
            us = self.to_int(kv.get("us"), 0)
            err = self.to_float(kv.get("error_rpm"), 0.0)
            target = self.to_float(kv.get("target_rpm"), 0.0)
            self.auto_status.set(f"Endpoint auto {endpoint}: active={active} us={us} error={err:.1f} RPM target={target:.0f}")
            if endpoint == "0%" and us:
                self._set_config_field_if_idle(self.ff0_us_var, str(us), "0% throttle us")
            elif endpoint == "100%" and us:
                self._set_config_field_if_idle(self.ff100_us_var, str(us), "100% throttle us")
        elif line.startswith("CFG "):
            self.parse_config_line(line[4:])
        elif line.startswith("HALLCAL "):
            self.parse_hall_cal_status_line(line[8:])
        elif line.startswith("OK PID"):
            self.tuning_status.set("Bridge accepted PID CAN frames")
        elif line.startswith("OK FFCOMMIT"):
            self.calibration_status.set("Bridge accepted atomic feedforward COMMIT; waiting for controller pull-back verification")
        elif line.startswith("OK FFRESET"):
            self.calibration_status.set("Bridge accepted feedforward reset; requesting selected-engine verification")
        elif line.startswith("OK FFCANCEL"):
            self.calibration_status.set("Bridge cancelled the staged feedforward model; active engine model was unchanged")
        elif line.startswith(("OK FFMODEL", "OK FFRPM", "OK FFCOEFF", "OK FFPOINT")):
            self.calibration_status.set("Feedforward model frame staged by bridge; engine remains unchanged until FFCOMMIT")
        elif line.startswith("OK FF"):
            self.tuning_status.set("Bridge accepted legacy feedforward CAN frame")
        elif line.startswith("OK STARTCFG"):
            self.tuning_status.set("Bridge accepted startup throttle config; controller will save it to FRAM")
        elif line.startswith("OK PWMTEST"):
            self.board_status.set("Bridge accepted direct throttle-PWM test command")
        elif line.startswith("OK THRESH"):
            self.tuning_status.set("Bridge accepted RPM sensor threshold CAN frame; controller will save it to FRAM")
        elif line.startswith("OK HALLCAL_STOP"):
            self.hall_auto_cal_status.set("Hall auto-cal stop accepted")
        elif line.startswith("OK HALLCAL"):
            self.hall_auto_cal_status.set("Hall auto-cal command accepted; hold the external spinner at the target RPM")
        elif line.startswith("OK RATE"):
            self.rate_status.set("Bridge accepted live telemetry-rate CAN frame")
        elif line.startswith("OK GETCFG"):
            self.tuning_status.set("Config request accepted; waiting for selected motor CFG frames")
        elif line.startswith("OK PWMBYPASS_STOP"):
            self.board_status.set("Manual PWM bypass stopped on selected motor")
        elif line.startswith("OK PWMBYPASS"):
            self.board_status.set("Manual PWM bypass command accepted and maintained for selected motor")
        elif line.startswith("ERR BYPASS_REQUIRES_ARM"):
            self.manual_pwm_bypass_var.set(False)
            self.board_status.set("PWM bypass rejected by bridge: select and ARM the motor first")
        elif line.startswith(("ERR BAD_FFMODEL", "ERR BAD_FFRPM", "ERR BAD_FFCOEFF", "ERR BAD_FFPOINT")):
            self.calibration_status.set(f"Feedforward transfer rejected by bridge: {line}")
        elif line.startswith("ERR BUSY") and any(token in line for token in ("FFMODEL", "FFRPM", "FFCOEFF", "FFPOINT", "FFCOMMIT", "FFRESET", "FFCANCEL")):
            self.calibration_status.set(f"Feedforward transfer could not be sent: {line}")

    def parse_board_line(self, text: str) -> None:
        kv = self.parse_kv(text)
        board_id = self.to_int(kv.get("id"), -1)
        if board_id < 0:
            return
        info = self.boards.get(board_id, BoardInfo(board_id=board_id))
        info.rpm = self.to_int(kv.get("rpm"), info.rpm)
        info.state = self.to_int(kv.get("state"), info.state)
        info.flags = self.to_int(kv.get("flags"), info.flags)
        info.last_rx_monotonic = time.monotonic()
        self.boards[board_id] = info
        self._selected_board_board_telemetry_fallback(info)
        self.refresh_board_combo_values()


    def _selected_board_board_telemetry_fallback(self, info: BoardInfo) -> None:
        """Use selected-board BOARD announcements as a live telemetry fallback.

        The controller publishes full TEL A/B/C only when it believes it is
        selected. During selection/bridge hiccups we can still receive BOARD
        announcements, which carry enough RPM/state/armed information to keep
        the live values and graph alive. Do not let these overwrite fresh TEL A
        samples, because TEL A has the exact output PWM and command-link bits.
        """
        if self.selected_board_id is None or info.board_id != self.selected_board_id:
            return

        now = time.monotonic()
        tel_a_age = now - self.last_tel_a_monotonic if self.last_tel_a_monotonic else 1e9
        if tel_a_age < 0.30:
            return

        telemetry_flags = 0
        if info.armed:
            telemetry_flags |= 0x01
        if info.debug_seen:
            telemetry_flags |= 0x02
        if info.selected:
            telemetry_flags |= 0x08

        self.telemetry.rpm = float(info.rpm)
        self.telemetry.state = info.state
        self.telemetry.flags = telemetry_flags
        self.telemetry.last_rx_monotonic = now

        # BOARD announcements are slower than TEL A. Still record them so the
        # graph does not sit at zero/stale when selecting an engine suppresses
        # or delays TEL A. Limit duplicate BOARD fallback samples.
        if now - self.last_selected_board_fallback_monotonic >= 0.20:
            self.last_selected_board_fallback_monotonic = now
            self.record_rpm_sample()

    def refresh_board_combo_values(self) -> None:
        now = time.monotonic()
        live_ids = [bid for bid, b in sorted(self.boards.items()) if now - b.last_rx_monotonic < 3.0]
        labels = [str(bid) for bid in live_ids]
        if hasattr(self, "board_combo"):
            self.board_combo.configure(values=labels)
        if self.selected_board_id is None and live_ids:
            self.selected_board_id = live_ids[0]
            self.selected_board_var.set(str(live_ids[0]))
        selected_txt = f"selected={self.selected_board_id}" if self.selected_board_id is not None else "none selected"
        self.board_status.set(f"Discovered {len(live_ids)} live board(s); {selected_txt}")

    def scan_boards(self) -> None:
        self.send_line("PROBE")
        self.board_status.set("Probe sent; waiting for BOARD announcements")

    def select_board_from_gui(self) -> None:
        text = self.selected_board_var.get().strip()
        if not text:
            messagebox.showerror("No board selected", "Scan first, then choose a board ID.")
            return
        try:
            board_id = int(text, 0)
        except ValueError:
            messagebox.showerror("Bad board ID", "Board ID must be a number.")
            return
        if self.ff_cal_mode != "idle" and self.ff_cal_board_id != board_id:
            self.stop_feedforward_calibration(capture=False, silent=True)
            self.calibration_status.set("Calibration aborted before changing the selected engine")
        if self.selected_board_id is not None and self.selected_board_id != board_id and self.serial_worker.is_connected():
            self.send_line("FFCANCEL")  # cancel any incomplete transaction on the previously selected engine
            self.ff_pending_commit_snapshot = None
        self.selected_board_id = board_id

        # A board switch invalidates the previous TEL A stream. Clear the TEL-A
        # freshness marker so the selected board's BOARD announcement can keep
        # the live values/graph alive immediately while the controller latches
        # the new selection and starts sending full TEL A/B/C again.
        self.last_tel_a_monotonic = 0.0
        existing = self.boards.get(board_id)
        if existing is not None:
            self._selected_board_board_telemetry_fallback(existing)

        # Send selection more than once and force a probe heartbeat. The motor
        # controller also treats the selected ID inside PROBE as a redundant
        # selection heartbeat, so telemetry recovers even if one SELECT frame is
        # dropped on a busy CAN bus.
        self.send_line(f"SELECT {board_id}")
        self.after(40, lambda bid=board_id: self.send_line(f"SELECT {bid}"))
        self.after(100, lambda bid=board_id: self.send_line(f"SELECT {bid}"))
        self.after(140, lambda: self.send_line("PROBE"))
        self.board_status.set(f"Selected board {board_id}; TEL A/B/C should resume, with BOARD-announcement fallback while selection latches")

    def identify_selected_board(self) -> None:
        if not self.reselect_selected_board():
            return
        self.send_line("IDENTIFY")
        self.board_status.set("Selected engine is blinking/beeping; press Stop blink/beep once you found it")

    def stop_identify_selected_board(self) -> None:
        if not self.ensure_board_selected():
            return
        # Send a few times because this is a human locator/safety convenience
        # command and a single CAN frame can be missed. Firmware treats it as
        # idempotent and global.
        self.send_line("STOP_IDENTIFY")
        self.after(80, lambda: self.send_line("STOP_IDENTIFY"))
        self.after(180, lambda: self.send_line("STOP_IDENTIFY"))
        self.board_status.set("Stopped the manual beep/flash locator pattern")

    def open_throttle_pwm_popup(self) -> None:
        if not self.ensure_board_selected():
            return
        win = tk.Toplevel(self)
        win.title("Direct throttle PWM / bypass test")
        win.geometry("560x470")
        win.minsize(500, 430)
        win.transient(self)
        # Keep this window non-modal so the main ARM/DISARM controls remain
        # available while setting up an armed/running bypass test.
        win.protocol("WM_DELETE_WINDOW", lambda: self.stop_manual_pwm_test_popup(win))

        frame = ttk.Frame(win, padding=12)
        frame.pack(fill="both", expand=True)
        for col in range(3):
            frame.columnconfigure(col, weight=1)

        ttk.Label(
            frame,
            text=(
                "Normal PWM test powers the relay/servo rail only while disarmed/stationary. "
                "Bypass mode is selected-engine only and drives exact throttle PWM while the engine is armed/running, bypassing feedforward and PID."
            ),
            wraplength=520,
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        ttk.Label(frame, text="0% throttle us").grid(row=1, column=0, sticky="w")
        ttk.Label(frame, text="100% throttle us").grid(row=1, column=1, sticky="w")
        ttk.Label(frame, text="Start throttle us").grid(row=1, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.ff0_us_var, width=10).grid(row=2, column=0, sticky="ew", padx=(0, 6))
        ttk.Entry(frame, textvariable=self.ff100_us_var, width=10).grid(row=2, column=1, sticky="ew", padx=(0, 6))
        ttk.Entry(frame, textvariable=self.start_us_var, width=10).grid(row=2, column=2, sticky="ew")

        ttk.Label(frame, text="Manual PWM-test hold ms").grid(row=3, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Entry(frame, textvariable=self.manual_pwm_hold_ms_var, width=10).grid(row=4, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(frame, text="Apply start config to FRAM", command=self.apply_start_config_only).grid(row=4, column=1, columnspan=2, sticky="ew")

        ttk.Checkbutton(
            frame,
            text="Bypass feedforward/PID for selected armed/running motor",
            variable=self.manual_pwm_bypass_var,
            command=self.manual_pwm_bypass_toggled,
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(12, 0))

        ttk.Label(frame, text="Live bypass/manual PWM slider").grid(row=6, column=0, columnspan=2, sticky="w", pady=(12, 0))
        ttk.Label(frame, textvariable=self.manual_pwm_slider_label, font=("TkDefaultFont", 11, "bold")).grid(row=6, column=2, sticky="e", pady=(12, 0))
        ttk.Scale(
            frame,
            from_=2000.0,
            to=1000.0,
            variable=self.manual_pwm_slider_us_var,
            command=self._manual_pwm_slider_changed,
        ).grid(row=7, column=0, columnspan=3, sticky="ew", pady=(4, 0))
        ttk.Button(frame, text="Send slider PWM now", command=lambda: self.send_manual_pwm_slider(live=False)).grid(row=8, column=0, columnspan=3, sticky="ew", pady=(8, 0))

        ttk.Button(frame, text="Set to 0% PWM", command=lambda: self.send_manual_pwm_test_from_var(self.ff0_us_var)).grid(row=9, column=0, sticky="ew", pady=(12, 0), padx=(0, 6))
        ttk.Button(frame, text="Set to 100% PWM", command=lambda: self.send_manual_pwm_test_from_var(self.ff100_us_var)).grid(row=9, column=1, sticky="ew", pady=(12, 0), padx=(0, 6))
        ttk.Button(frame, text="Set to START PWM", command=lambda: self.send_manual_pwm_test_from_var(self.start_us_var)).grid(row=9, column=2, sticky="ew", pady=(12, 0))
        ttk.Button(frame, text="Stop bypass only", command=self.stop_pwm_bypass).grid(row=10, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        ttk.Button(frame, text="Stop / safe disarm", command=lambda: self.stop_manual_pwm_test_popup(win)).grid(row=11, column=0, columnspan=3, sticky="ew", pady=(8, 0))

    def stop_manual_pwm_test_popup(self, win: tk.Toplevel) -> None:
        self.send_line("PWMBYPASS_STOP")
        self.send_line("PWMTEST_STOP")
        self.send_line("CMD 0 0")
        self.command_armed = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.manual_pwm_bypass_var.set(False)
        self.board_status.set("Direct throttle PWM/bypass stopped; safe disarm sent")
        try:
            win.destroy()
        except tk.TclError:
            pass

    def _manual_pwm_slider_changed(self, _event: object = None) -> None:
        us = int(round(float(self.manual_pwm_slider_us_var.get())))
        self.manual_pwm_slider_label.set(f"{us} us")
        if self.manual_pwm_bypass_var.get():
            self.send_manual_pwm_slider(live=True)

    def manual_pwm_bypass_toggled(self) -> None:
        if self.manual_pwm_bypass_var.get():
            if not self.command_armed:
                self.manual_pwm_bypass_var.set(False)
                messagebox.showerror(
                    "Arm required",
                    "PWM bypass is only allowed for an already armed/running selected motor. Arm the motor first, then enable bypass.",
                )
                return
            # Refresh the normal armed command/link first. A controller that had
            # briefly timed out clears bypass as part of its disarm safety gate,
            # so the armed heartbeat must arrive before bypass is re-enabled.
            self.send_current_command()
            self.after(40, self._reassert_pwm_bypass_if_enabled)
            self.after(140, self._reassert_pwm_bypass_if_enabled)
        else:
            self.stop_pwm_bypass()

    def _reassert_pwm_bypass_if_enabled(self) -> None:
        # Delayed retries must never fall through to the disarmed PWMTEST path if
        # the user unchecked bypass before the timer fired.
        if self.manual_pwm_bypass_var.get() and self.command_armed:
            us = int(round(float(self.manual_pwm_slider_us_var.get())))
            if 1000 <= us <= 2000:
                self.send_pwm_bypass_us(us, live=False)

    def send_manual_pwm_slider(self, live: bool = False) -> None:
        us = int(round(float(self.manual_pwm_slider_us_var.get())))
        if not 1000 <= us <= 2000:
            return
        if self.manual_pwm_bypass_var.get():
            self.send_pwm_bypass_us(us, live=live)
        elif not live:
            self.send_manual_pwm_test_us(us)

    def send_pwm_bypass_us(self, pwm_us: int, live: bool = False) -> None:
        if not self.ensure_board_selected():
            return
        if not self.command_armed:
            self.manual_pwm_bypass_var.set(False)
            self.board_status.set("PWM bypass rejected: arm the selected motor first")
            return
        if live:
            now = time.monotonic()
            if now - self.last_manual_pwm_slider_send < 0.08:
                return
            self.last_manual_pwm_slider_send = now
        if self.selected_board_id is not None:
            self.send_line(f"SELECT {self.selected_board_id}")
        self.send_line(f"PWMBYPASS {pwm_us}")
        self.board_status.set(f"Selected motor PWM bypass maintained at {pwm_us} us; feedforward/PID bypassed")

    def stop_pwm_bypass(self) -> None:
        self.manual_pwm_bypass_var.set(False)
        if self.ensure_board_selected():
            self.send_line("PWMBYPASS_STOP")
            self.board_status.set("Selected motor PWM bypass stopped; normal feedforward/PID resumes")

    def send_manual_pwm_test_us(self, pwm_us: int) -> None:
        if not self.reselect_selected_board():
            return
        hold_ms = self._parse_manual_pwm_hold_ms_field()
        if hold_ms is None:
            return
        self.command_armed = False
        self._stop_gui_automated_flags()
        self.send_line("CMD 0 0")
        self.send_line("PWMTEST_STOP")
        self.send_line(f"PWMTEST {pwm_us} {hold_ms}")
        self.board_status.set(f"Direct throttle PWM test sent: {pwm_us} us for {hold_ms} ms. Starter stays off; aborts on RPM.")
        self.save_settings(silent=True)

    def _parse_servo_us_field(self, var: tk.StringVar, label: str) -> Optional[int]:
        try:
            value = int(float(var.get()))
        except ValueError:
            messagebox.showerror("Bad PWM", f"{label} must be a numeric microsecond value.")
            return None
        if not 1000 <= value <= 2000:
            messagebox.showerror("Bad PWM", f"{label} must stay inside 1000..2000 us.")
            return None
        return value

    def _parse_start_hold_ms_field(self) -> Optional[int]:
        try:
            value = int(float(self.start_hold_ms_var.get()))
        except ValueError:
            messagebox.showerror("Bad start hold", "Start hold time must be numeric milliseconds.")
            return None
        if not 0 <= value <= 60000:
            messagebox.showerror("Bad start hold", "Start hold time must be 0..60000 ms.")
            return None
        return value

    def _parse_manual_pwm_hold_ms_field(self) -> Optional[int]:
        try:
            value = int(float(self.manual_pwm_hold_ms_var.get()))
        except ValueError:
            messagebox.showerror("Bad PWM hold", "Manual PWM-test hold time must be numeric milliseconds.")
            return None
        if not 1 <= value <= 60000:
            messagebox.showerror("Bad PWM hold", "Manual PWM-test hold time must be 1..60000 ms.")
            return None
        return value

    def apply_start_config_only(self) -> None:
        if not self.reselect_selected_board():
            return
        start_us = self._parse_servo_us_field(self.start_us_var, "Start throttle us")
        hold_ms = self._parse_start_hold_ms_field()
        if start_us is None or hold_ms is None:
            return
        self._send_reliable_config(
            f"STARTCFG {start_us} {hold_ms}",
            [(self.start_us_var, float(start_us), "Start us"), (self.start_hold_ms_var, float(hold_ms), "Start hold ms")],
            f"Startup config sent: start={start_us} us, hold after RPM={hold_ms} ms.",
        )
        self.save_settings(silent=True)

    def send_manual_pwm_test_from_var(self, var: tk.StringVar) -> None:
        pwm_us = self._parse_servo_us_field(var, "Throttle PWM")
        if pwm_us is None:
            return
        self.manual_pwm_slider_us_var.set(float(pwm_us))
        self.manual_pwm_slider_label.set(f"{pwm_us} us")
        if self.manual_pwm_bypass_var.get():
            self.send_pwm_bypass_us(pwm_us, live=False)
        else:
            self.send_manual_pwm_test_us(pwm_us)

    def set_selected_board_id(self) -> None:
        if not self.ensure_board_selected():
            return
        assert self.selected_board_id is not None
        new_id = simpledialog.askinteger("Set persistent board ID", "New board ID saved to FRAM:", minvalue=1, maxvalue=2_147_483_647)
        if new_id is None:
            return
        self.send_line(f"SETID {self.selected_board_id} {new_id}")
        self.boards.pop(self.selected_board_id, None)
        self.selected_board_id = int(new_id)
        self.selected_board_var.set(str(new_id))
        self.refresh_board_combo_values()

    def panic_all_boards(self) -> None:
        self.command_armed = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.all_cal_active = False
        self.hall_auto_cal_active = False
        self.send_line("HALLCAL_STOP")
        self.send_line("PANIC_ALL")
        self.close_process_popup()
        self.command_status.set("PANIC ALL: all engines selected, auto-adjust stopped, zero throttle + disarm sent")

    def ensure_board_selected(self) -> bool:
        if self.selected_board_id is None:
            before = self.selected_board_id
            self.select_board_from_gui()
            return self.selected_board_id is not None and self.selected_board_id != before or self.selected_board_id is not None
        if self.selected_board_var.get().strip() != str(self.selected_board_id):
            self.select_board_from_gui()
        return self.selected_board_id is not None

    def reselect_selected_board(self) -> bool:
        # The bridge can be reset or PANIC_ALL can leave the bridge in broadcast
        # selection while the GUI still shows a normal board ID. Always re-send
        # SELECT before commands that must be accepted by exactly one controller.
        if not self.ensure_board_selected():
            return False
        assert self.selected_board_id is not None
        self.send_line(f"SELECT {self.selected_board_id}")
        return True

    @staticmethod
    def _hall_thresholds_from_min_max(min_raw: int, max_raw: int) -> tuple[int, int]:
        span = max(max_raw - min_raw, 0)
        low = int(min_raw + span * 0.35 + 0.5)
        high = int(min_raw + span * 0.65 + 0.5)
        high = max(min(high, 4095), 0)
        low = max(min(low, 4095), 0)
        if high <= low:
            high = min(low + 1, 4095)
            if high <= low and low > 0:
                low -= 1
        return high, low

    def start_hall_auto_cal(self) -> None:
        if not self.reselect_selected_board():
            return
        if not self.serial_worker.is_connected():
            messagebox.showerror("Bridge disconnected", "Connect the USB↔CAN bridge before starting Hall auto-cal.")
            return
        try:
            target = float(self.hall_cal_target_rpm_var.get())
        except ValueError:
            messagebox.showerror("Bad Hall auto-cal", "Hall target RPM must be numeric.")
            return
        if target <= 0.0 or target > 50000.0:
            messagebox.showerror("Bad Hall auto-cal", "Hall calibration target RPM must be in 1..50000.")
            return

        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.all_cal_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.command_armed = True
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.send_line("STOP_IDENTIFY")
        self.send_current_command()
        self.send_line(f"HALLCAL {target:.8g}")
        self.hall_auto_cal_active = True
        detail = (
            f"External spinner target: {target:.0f} RPM. The routine keeps running until the raw Hall signal "
            "has stable min/max and clean edge timing, or until you abort it. The board is armed to power "
            "the Hall sensor rail; starter stays off and throttle is held at idle."
        )
        self.hall_auto_cal_status.set("Hall auto-cal running: hold the spinner steady at the target RPM")
        self.show_process_popup("Hall sensor raw min/max auto-cal", detail)
        self.save_settings(silent=True)

    def stop_hall_auto_cal(self) -> None:
        self.hall_auto_cal_active = False
        if self.serial_worker.is_connected():
            self.send_line("HALLCAL_STOP")
            self.send_line("AUTO_STOP")
        self.command_armed = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.send_current_command()
        self.close_process_popup()
        self.hall_auto_cal_status.set("Hall auto-cal stopped; zero throttle + disarm sent")

    def parse_hall_cal_status_line(self, text: str) -> None:
        kv = self.parse_kv(text)
        status = kv.get("status", "UNKNOWN")
        progress = self.to_int(kv.get("progress"), 0)
        min_raw = self.to_int(kv.get("min"), 0)
        max_raw = self.to_int(kv.get("max"), 0)
        span = self.to_int(kv.get("span"), max_raw - min_raw if max_raw >= min_raw else 0)
        target = self.to_float(kv.get("target_rpm"), 0.0)

        if status == "RUNNING":
            quality = self.to_int(kv.get("quality"), progress)
            detail = f"Hall auto-cal running: quality={quality}%, raw min={min_raw}, max={max_raw}, span={span}, target={target:.0f} RPM. Keep spinner steady; it saves only after stable raw windows."
            self.hall_auto_cal_status.set(detail)
            if self.hall_auto_cal_active:
                self.update_process_popup(detail)
            return

        if status == "DONE_OK":
            high, low = self._hall_thresholds_from_min_max(min_raw, max_raw)
            updated_high = self._set_config_field_if_idle(self.hall_high_raw_var, str(high), "RPM high raw")
            updated_low = self._set_config_field_if_idle(self.hall_low_raw_var, str(low), "RPM low raw")
            msg = f"Hall auto-cal OK: min={min_raw}, max={max_raw}, span={span}; thresholds high={high}, low={low}. Saved to FRAM by controller."
            if not (updated_high and updated_low):
                msg += " Visible threshold field(s) were not overwritten because you are editing them."
            self.hall_auto_cal_status.set(msg)
            self.tuning_status.set(msg)

            # Only a Hall-calibration process that this GUI started may change
            # the manual ARM command. The controller continues to publish the
            # last Hall-cal status while inactive, so treating every old DONE_OK
            # packet as a command to disarm makes normal ARM drop out one
            # telemetry tick after pressing the button.
            if self.hall_auto_cal_active:
                self.hall_auto_cal_active = False
                self.command_armed = False
                self.manual_throttle.set(0.0)
                self._slider_changed()
                self.send_current_command()
                self.close_process_popup()
                self.save_settings(silent=True)
            return

        if status == "FAILED":
            msg = f"Hall auto-cal aborted/timeout: min={min_raw}, max={max_raw}, span={span}. It normally keeps running until clean, so this usually means abort/stop or an optional CLI timeout."
            self.hall_auto_cal_status.set(msg)

            # Same guard as DONE_OK: stale inactive Hall-cal FAILED telemetry
            # must not disarm the normal manual ARM path.
            if self.hall_auto_cal_active:
                self.hall_auto_cal_active = False
                self.command_armed = False
                self.manual_throttle.set(0.0)
                self._slider_changed()
                self.send_current_command()
                self.close_process_popup()

    def start_endpoint_auto_adjust(self, endpoint_label: str, show_popup: bool = True) -> None:
        if not self.reselect_selected_board():
            return
        try:
            rate = int(float(self.endpoint_auto_rate_var.get()))
            start_us = int(float(self.endpoint_auto_start_us_var.get()))
            target = float(self.ff0_rpm_var.get() if endpoint_label == "0%" else self.ff100_rpm_var.get())
        except ValueError:
            messagebox.showerror("Bad endpoint auto-adjust", "Target RPM, adjustment rate, and start PWM must be numeric.")
            return
        if rate <= 0 or rate > 200:
            messagebox.showerror("Bad endpoint auto-adjust", "Rate should be 1..200 servo microseconds per second.")
            return
        if start_us < 1000 or start_us > 2000:
            messagebox.showerror("Bad endpoint auto-adjust", "Start PWM should be inside the hard servo range, normally 1000..2000 us. Use 1700 us as the normal search start.")
            return
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.hall_auto_cal_active = False
        self.send_line("HALLCAL_STOP")
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.command_armed = True
        self.manual_throttle.set(0.0 if endpoint_label == "0%" else 100.0)
        self._slider_changed()

        # Keep the RPM target fixed from the user/saved field. The board must
        # change only the endpoint PWM value; it should never overwrite the RPM
        # target with a measured/derived value during endpoint auto-adjust.
        cmd = f"AUTO0 {target:.8g} {rate} {start_us}" if endpoint_label == "0%" else f"AUTO100 {target:.8g} {rate} {start_us}"
        self.send_line(cmd)
        self.send_current_command()
        self.auto_status.set(f"Auto-adjusting selected {endpoint_label} PWM from {start_us} us toward fixed target {target:.0f} RPM at ≤{rate} us/s")
        if show_popup:
            self.show_process_popup(
                f"selected {endpoint_label} endpoint auto-adjust",
                f"Fixed target {target:.0f} RPM, starting from {start_us} us, max rate {rate} us/s. Press ABORT PROCESS to stop tuning and return to 0% throttle without disarming.",
            )

    def stop_endpoint_auto_adjust(self, close_popup: bool = True) -> None:
        # Stop the tuning loop only. Do not disarm; command the selected engine
        # to 0% throttle and preserve the current ARM state.
        self.send_line("AUTO_STOP")
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.send_current_command()
        self.after(80, self.send_current_command)
        self.after(180, self.send_current_command)
        if close_popup:
            self.close_process_popup()
        self.auto_status.set("Endpoint auto-adjust stopped; command moved to 0% throttle, ARM state preserved")
        self.command_status.set("Auto-adjust stopped: sending 0% throttle without disarming")

    def start_selected_engine_autocalibrate(self) -> None:
        if not self.reselect_selected_board():
            return
        if not self.serial_worker.is_connected():
            messagebox.showerror("Bridge disconnected", "Connect the USB↔CAN bridge before autocalibrating.")
            return
        assert self.selected_board_id is not None
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.all_cal_active = False
        self.hall_auto_cal_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.send_line("HALLCAL_STOP")
        self.send_line("AUTO_STOP")
        self.command_armed = True
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.send_current_command()
        self.selected_autocal_active = True
        self.selected_autocal_board_id = self.selected_board_id
        self.selected_autocal_phase = "warmup"
        self.selected_autocal_phase_deadline = time.monotonic() + 120.0
        detail = "Phase 1/3: armed at 0% for 2:00 warmup. Then auto-adjust 0% for 1:30 and 100% for 1:30."
        self.auto_status.set(detail)
        self.command_status.set("Selected-engine autocalibration active; heartbeat remains armed")
        self.show_process_popup("selected engine autocalibrate", detail)

    def update_selected_engine_autocalibrate(self) -> None:
        if not self.selected_autocal_active:
            return
        now = time.monotonic()
        board_txt = f"Board {self.selected_autocal_board_id}" if self.selected_autocal_board_id is not None else "Selected board"

        if self.selected_autocal_phase == "warmup":
            self.command_armed = True
            self.manual_throttle.set(0.0)
            self._slider_changed()
            remaining = max(self.selected_autocal_phase_deadline - now, 0.0)
            detail = f"{board_txt}: warmup at 0% throttle, {remaining:0.0f} s remaining"
            self.auto_status.set(detail)
            self.update_process_popup(detail)
            if now >= self.selected_autocal_phase_deadline:
                self.start_endpoint_auto_adjust("0%", show_popup=False)
                self.selected_autocal_phase = "auto0"
                self.selected_autocal_phase_deadline = now + 90.0
            return

        if self.selected_autocal_phase == "auto0":
            remaining = max(self.selected_autocal_phase_deadline - now, 0.0)
            detail = f"{board_txt}: auto-adjusting 0% endpoint, {remaining:0.0f} s remaining"
            self.auto_status.set(detail)
            self.update_process_popup(detail)
            if now >= self.selected_autocal_phase_deadline:
                self.stop_endpoint_auto_adjust(close_popup=False)
                self.start_endpoint_auto_adjust("100%", show_popup=False)
                self.selected_autocal_phase = "auto100"
                self.selected_autocal_phase_deadline = now + 90.0
            return

        if self.selected_autocal_phase == "auto100":
            remaining = max(self.selected_autocal_phase_deadline - now, 0.0)
            detail = f"{board_txt}: auto-adjusting 100% endpoint, {remaining:0.0f} s remaining"
            self.auto_status.set(detail)
            self.update_process_popup(detail)
            if now >= self.selected_autocal_phase_deadline:
                self.stop_endpoint_auto_adjust(close_popup=False)
                self.command_armed = True
                self.manual_throttle.set(0.0)
                self._slider_changed()
                self.send_current_command()
                self.selected_autocal_active = False
                self.selected_autocal_phase = "idle"
                self.auto_status.set(f"{board_txt}: autocalibration complete; engine remains armed at 0% throttle")
                self.command_status.set("Autocalibration complete: selected engine held at 0%, still armed for manual testing")
                self.close_process_popup("selected engine autocalibrate")

    def start_all_boards_endpoint_auto(self, endpoint_label: str) -> None:
        live_ids = [bid for bid, b in sorted(self.boards.items()) if time.monotonic() - b.last_rx_monotonic < 3.0]
        if not live_ids:
            messagebox.showerror("No boards", "Scan/discover the engine boards first.")
            return
        try:
            float(self.endpoint_auto_duration_var.get())
        except ValueError:
            messagebox.showerror("Bad dwell", "All-board dwell time must be numeric seconds.")
            return
        self.all_cal_active = True
        self.all_cal_queue = live_ids
        self.all_cal_endpoint_label = endpoint_label
        self.all_cal_phase = "next"
        self.all_cal_phase_deadline = time.monotonic()
        self.auto_status.set(f"Clockwise all-board {endpoint_label} auto-adjust queued for {len(live_ids)} engines")
        self.show_process_popup(
            f"clockwise all-board {endpoint_label} endpoint auto-adjust",
            f"Queued {len(live_ids)} engine(s). Each selected engine will identify, auto-adjust, then return to 0% throttle. Press ABORT PROCESS to stop all.",
        )

    def update_all_boards_endpoint_auto(self) -> None:
        if not self.all_cal_active:
            return
        now = time.monotonic()
        if now < self.all_cal_phase_deadline:
            return
        if self.all_cal_phase == "next":
            if not self.all_cal_queue:
                self.all_cal_active = False
                self.stop_endpoint_auto_adjust()
                self.auto_status.set("Clockwise all-board endpoint auto-adjust complete")
                self.close_process_popup()
                return
            bid = self.all_cal_queue.pop(0)
            self.selected_board_id = bid
            self.selected_board_var.set(str(bid))
            self.send_line(f"SELECT {bid}")
            self.send_line("IDENTIFY")
            self.all_cal_phase = "run"
            self.all_cal_phase_deadline = now + 3.0
            status = f"Board {bid}: blink/beep identify before {self.all_cal_endpoint_label} auto-adjust"
            self.auto_status.set(status)
            self.update_process_popup(status)
            return
        if self.all_cal_phase == "run":
            # The identify tone is only a pre-start locator for the selected
            # engine. Stop it before the automated throttle/RPM adjustment begins.
            self.send_line("STOP_IDENTIFY")
            self.start_endpoint_auto_adjust(self.all_cal_endpoint_label, show_popup=False)
            try:
                dwell = max(float(self.endpoint_auto_duration_var.get()), 1.0)
            except ValueError:
                dwell = 15.0
            self.all_cal_phase = "stop"
            self.all_cal_phase_deadline = now + dwell
            self.update_process_popup(f"Board {self.selected_board_id}: auto-adjust running for {dwell:.1f} s")
            return
        if self.all_cal_phase == "stop":
            self.stop_endpoint_auto_adjust()
            self.show_process_popup(
                f"clockwise all-board {self.all_cal_endpoint_label} endpoint auto-adjust",
                "Moving to the next engine...",
            )
            self.all_cal_phase = "next"
            self.all_cal_phase_deadline = now + 1.0

    def _textvariable_widget_is_focused(self, var: tk.Variable) -> bool:
        """Return True while the user is actively editing an Entry/Combobox bound to var.

        Live CAN/FRAM telemetry must not overwrite a field while the cursor is in
        that field. Tk stores the variable name as a string on the widget, so this
        also protects popup entries that share the same StringVar as the main panel.

        Important: ttk Combobox dropdowns can temporarily report focus paths such
        as ``.foo.popdown`` that no longer exist when focus_get() resolves them,
        especially on newer Tk/Python. Treat that as "not editing" instead of
        throwing, otherwise one telemetry update stops the whole GUI telemetry loop.
        """
        try:
            focus = self.focus_get()
        except (tk.TclError, KeyError):
            return False
        if focus is None:
            return False

        wanted = str(var)
        widget = focus
        while widget is not None:
            try:
                textvariable = widget.cget("textvariable")
            except (tk.TclError, AttributeError, KeyError):
                textvariable = ""
            if str(textvariable) == wanted:
                return True
            try:
                widget = widget.master
            except (AttributeError, KeyError):
                widget = None
        return False

    def _mark_main_config_dirty(self, var: tk.StringVar, label: str) -> None:
        key = str(var)
        self.config_dirty_vars.add(key)
        self.config_pending_values.pop(key, None)
        self.config_pending_boards.pop(key, None)
        self.config_pull_overwrite_vars.discard(key)
        widget = self.config_entry_widgets.get(key)
        if widget is not None:
            try:
                widget.configure(style="PendingConfig.TEntry")
                ttk.Style(self).configure("PendingConfig.TEntry", fieldbackground="#fff0b3")
            except tk.TclError:
                pass
        self.tuning_status.set(f"UNSENT LOCAL EDIT in {label}. Press Send; controller telemetry will not overwrite it.")

    @staticmethod
    def _config_values_match(expected: float, incoming: object) -> bool:
        try:
            actual = float(incoming)
        except (TypeError, ValueError):
            return False
        tolerance = max(1.0e-5, abs(expected) * 1.0e-5)
        return abs(actual - expected) <= tolerance

    def _clear_main_config_pending(self, var: tk.StringVar) -> None:
        key = str(var)
        self.config_dirty_vars.discard(key)
        self.config_pending_values.pop(key, None)
        self.config_pending_boards.pop(key, None)
        self.config_pull_overwrite_vars.discard(key)
        widget = self.config_entry_widgets.get(key)
        if widget is not None:
            try:
                widget.configure(style="TEntry")
            except tk.TclError:
                pass

    def _begin_explicit_config_pull(self) -> None:
        """Allow one incoming value per main field to replace the current local value.

        Typing in a field after pressing Pull removes that field from this set, so
        a delayed response cannot overwrite a new edit.
        """
        self.config_pull_overwrite_vars = set(self.config_entry_widgets)

        def expire() -> None:
            self.config_pull_overwrite_vars.clear()

        self.after(2500, expire)

    def _set_config_field_if_idle(self, var: tk.StringVar, value: object, label: str = "config field") -> bool:
        """Apply controller values without destroying pending user edits.

        A field is updated when it is clean, during an explicit Pull, or when the
        incoming value confirms the value that was just sent. A stale response is
        ignored even after keyboard focus moves to the Send button.
        """
        key = str(var)
        force_pull = key in self.config_pull_overwrite_vars
        pending = self.config_pending_values.get(key)
        pending_board = self.config_pending_boards.get(key)
        board_matches = pending_board is None or pending_board == self.selected_board_id

        if pending is not None and board_matches:
            if self._config_values_match(pending, value):
                var.set(str(value))
                self._clear_main_config_pending(var)
                self.tuning_status.set(f"Controller confirmed {label} = {value}.")
                return True
            if not force_pull:
                self.tuning_status.set(
                    f"Kept pending {label} = {var.get()}; ignored stale controller value {value}. "
                    "Automatic retries/verification are still running."
                )
                return False

        if not force_pull and (key in self.config_dirty_vars or self._textvariable_widget_is_focused(var)):
            self.tuning_status.set(f"Kept your local edit in {label}; use Send or explicit Pull to replace it.")
            return False

        var.set(str(value))
        if force_pull:
            self._clear_main_config_pending(var)
        return True

    def _send_reliable_config(
        self,
        command: str,
        expected: list[tuple[tk.StringVar, float, str]],
        status: str,
    ) -> bool:
        """Send an idempotent selected-engine config command with retries and pull-back verification."""
        if self.selected_board_id is None or not self.serial_worker.is_connected():
            return False
        board_id = self.selected_board_id
        for var, value, _label in expected:
            key = str(var)
            self.config_dirty_vars.add(key)
            self.config_pending_values[key] = float(value)
            self.config_pending_boards[key] = board_id
            widget = self.config_entry_widgets.get(key)
            if widget is not None:
                try:
                    ttk.Style(self).configure("PendingConfig.TEntry", fieldbackground="#fff0b3")
                    widget.configure(style="PendingConfig.TEntry")
                except tk.TclError:
                    pass

        def transmit() -> None:
            if (
                self.serial_worker.is_connected()
                and self.selected_board_id == board_id
                and any(self.config_pending_boards.get(str(v)) == board_id for v, _x, _l in expected)
            ):
                self.send_line(f"SELECT {board_id}")
                self.send_line(command)

        def verify() -> None:
            if self.serial_worker.is_connected() and self.selected_board_id == board_id:
                self.ff_model_pull_requested = False
                self.ff_model_receive = {"coefficients": {}, "points": {}, "board_id": board_id}
                self.send_line(f"SELECT {board_id}")
                self.send_line("GETCFG")

        transmit()
        self.after(100, transmit)
        self.after(260, transmit)
        self.after(430, verify)
        self.after(950, verify)
        self.tuning_status.set(status + " Waiting for controller confirmation; stale values are locked out.")
        return True

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

    def _stop_gui_automated_flags(self) -> None:
        if self.ff_cal_mode != "idle":
            self.stop_feedforward_calibration(capture=False, silent=True)
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.all_cal_active = False
        self.selected_autocal_active = False
        self.selected_autocal_phase = "idle"
        self.hall_auto_cal_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)

    def arm(self) -> None:
        if not self.reselect_selected_board():
            messagebox.showerror("No board selected", "Select the engine board before arming.")
            return

        # Do not send HALLCAL_STOP/AUTO_STOP before ARM. The Hall calibration
        # path worked because it powered the board first; the old ARM path sent
        # cleanup frames first, which could race the ARM command and make the
        # controller appear to disarm immediately. A rising normal ARM command
        # now tells the controller firmware to exit special debug modes itself.
        self._stop_gui_automated_flags()
        self.command_armed = True
        self.command_status.set("ARM command active; selected-board heartbeat transmitting while connected")
        self.send_current_command()
        self.after(50, self.send_current_command)
        self.after(120, self.send_current_command)
        self.after(250, self.send_current_command)

    def disarm(self) -> None:
        # Cancel bypass before the disarmed CMD heartbeat. This prevents a stale
        # checked box from automatically re-enabling exact PWM after a later ARM.
        if self.manual_pwm_bypass_var.get():
            self.manual_pwm_bypass_var.set(False)
            self.send_line("PWMBYPASS_STOP")
        self.command_armed = False
        self._stop_gui_automated_flags()
        self.send_line("HALLCAL_STOP")
        self.send_line("AUTO_STOP")
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.command_status.set("DISARM command active")
        self.sweep_status.set("Sweep inactive")
        self.square_status.set("Square wave inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")
        self.close_process_popup()
        self.send_current_command()

    def panic_disarm(self) -> None:
        self.disarm()
        self.send_line("HALLCAL_STOP")
        self.send_line("PANIC_ALL")
        self.send_line("CMD 0 0")
        self.close_process_popup()
        self.command_status.set("PANIC: zero throttle + disarm sent, plus PANIC_ALL broadcast")

    def send_current_command(self) -> Optional[float]:
        if not self.serial_worker.is_connected():
            return None
        if self.command_armed and self.selected_board_id is not None:
            self.send_line(f"SELECT {self.selected_board_id}")
        throttle = self.manual_throttle.get()
        if not self.send_line(f"CMD {1 if self.command_armed else 0} {throttle:.3f}"):
            return None

        # Use the timestamp of the completed serial write, not the ideal square-wave
        # phase time. This makes the plotted command edges match the actual CMD
        # heartbeat that left the GUI. Controller TEL A output PWM is timestamped
        # separately when it is received, so transport/controller delay remains visible.
        sent_at = time.monotonic()
        self.command_monotonic_times.append(sent_at)
        self.command_throttle_values.append(throttle)
        return sent_at

    def command_heartbeat_loop(self) -> None:
        if self.serial_worker.is_connected():
            self.update_active_pattern_value()
            self.send_current_command()
            # Maintain bypass as a mode rather than a one-shot CAN command. This
            # also recovers after a dropped bypass frame or brief selection/link
            # interruption. The bridge firmware independently maintains it too.
            if self.manual_pwm_bypass_var.get() and self.command_armed:
                now = time.monotonic()
                if now - self.last_manual_pwm_bypass_heartbeat >= 0.25:
                    self.last_manual_pwm_bypass_heartbeat = now
                    self.send_pwm_bypass_us(int(round(float(self.manual_pwm_slider_us_var.get()))), live=False)
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
        detail = f"Averaging {endpoint_label} feedforward RPM at {target_pct:.0f}% throttle for 10.0 s.{arm_note}"
        self.endpoint_average_status.set(detail)
        self.show_process_popup(f"{endpoint_label} endpoint RPM average", detail)

    def update_endpoint_average(self) -> None:
        if not self.endpoint_average_active:
            return
        elapsed = max(time.monotonic() - self.endpoint_average_started, 0.0)
        remaining = max(self.endpoint_average_duration_s - elapsed, 0.0)
        detail = f"Averaging {self.endpoint_average_label} feedforward RPM: {remaining:0.1f} s left, {len(self.endpoint_average_samples)} RPM sample(s) captured"
        self.endpoint_average_status.set(detail)
        self.update_process_popup(detail)
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
            self.close_process_popup()
            self.endpoint_average_status.set("Endpoint averaging finished, but no RPM samples were received; feedforward field was not changed.")
            messagebox.showerror("No RPM samples", "No TEL A RPM samples arrived during the 10-second averaging window.")
            return

        avg_rpm = sum(samples) / len(samples)
        if label == "0%":
            updated = self._set_config_field_if_idle(self.ff0_rpm_var, f"{avg_rpm:.3f}", "0% RPM")
        else:
            updated = self._set_config_field_if_idle(self.ff100_rpm_var, f"{avg_rpm:.3f}", "100% RPM")

        if not updated:
            self.endpoint_average_status.set(
                f"{label} averaging measured {avg_rpm:.2f} RPM, but did not overwrite the focused RPM field. Press Apply manually when ready."
            )
            self.close_process_popup()
            return

        self.endpoint_average_status.set(
            f"{label} feedforward RPM auto-set to {avg_rpm:.2f} RPM from {len(samples)} sample(s); applying feedforward update."
        )
        self.apply_tuning()
        self.save_settings(silent=True)
        self.close_process_popup()

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
        if was_active:
            self.close_process_popup()
        if was_active and not silent:
            suffix = "; throttle forced to 0%" if zero_throttle else ""
            self.endpoint_average_status.set(f"Endpoint RPM averaging cancelled{suffix}.")
        elif not was_active and zero_throttle and not silent:
            self.endpoint_average_status.set("No endpoint averaging was active; throttle forced to 0%.")

    def get_configs_selected_motor(self) -> None:
        if not self.reselect_selected_board():
            return
        assert self.selected_board_id is not None
        if self.ff_model_dirty and self.calibration_window is not None and self.calibration_window.winfo_exists():
            if not messagebox.askyesno(
                "Discard local feedforward edits?",
                "Pull all configs will replace the local calibration-window model for the selected engine. Continue?",
            ):
                return
        self._begin_explicit_config_pull()
        self.ff_model_pull_requested = True
        self.ff_model_receive = {"coefficients": {}, "points": {}, "board_id": self.selected_board_id}
        self.send_line("GETCFG")
        self.tuning_status.set(f"Requested runtime/FRAM config from selected motor {self.selected_board_id}")

    def parse_config_line(self, text: str) -> None:
        parts = text.split(maxsplit=1)
        kind = parts[0] if parts else ""
        rest = parts[1] if len(parts) > 1 else ""
        kv = self.parse_kv(rest)
        changed: list[str] = []
        cfg_board_id = self.to_int(kv.get("selected"), self.selected_board_id if self.selected_board_id is not None else -1)

        def ensure_model_rx() -> dict[str, object]:
            if cfg_board_id < 0:
                return self.ff_model_receive
            current_board = self.ff_model_receive.get("board_id")
            if current_board is None or int(current_board) != cfg_board_id:
                self.ff_model_receive = {"coefficients": {}, "points": {}, "board_id": cfg_board_id}
            return self.ff_model_receive

        if kind == "PID":
            if "kp" in kv and self._set_config_field_if_idle(self.kp_var, kv["kp"], "Kp"):
                changed.append("Kp")
            if "ki" in kv and self._set_config_field_if_idle(self.ki_var, kv["ki"], "Ki"):
                changed.append("Ki")
        elif kind == "PID2":
            if "kd" in kv and self._set_config_field_if_idle(self.kd_var, kv["kd"], "Kd"):
                changed.append("Kd")
            if "limit" in kv and self._set_config_field_if_idle(self.limit_var, kv["limit"], "PID limit"):
                changed.append("PID limit")
        elif kind == "FF0":
            rx = ensure_model_rx()
            if "rpm" in kv:
                rx["idle_rpm"] = self.to_float(kv.get("rpm"), 0.0)
                if cfg_board_id >= 0:
                    self.ff0_rpm_confirmed_by_board[cfg_board_id] = float(rx["idle_rpm"])
                if self._set_config_field_if_idle(self.ff0_rpm_var, kv["rpm"], "0% RPM"):
                    changed.append("0% RPM")
            if "us" in kv:
                rx["idle_us"] = self.to_int(kv.get("us"), 0)
                if self._set_config_field_if_idle(self.ff0_us_var, kv["us"], "0% us"):
                    changed.append("0% us")
            self._try_finalize_ff_model_pull()
        elif kind == "FF100":
            rx = ensure_model_rx()
            if "rpm" in kv:
                rx["max_rpm"] = self.to_float(kv.get("rpm"), 0.0)
                if self._set_config_field_if_idle(self.ff100_rpm_var, kv["rpm"], "100% RPM"):
                    changed.append("100% RPM")
            if "us" in kv:
                rx["max_us"] = self.to_int(kv.get("us"), 0)
                if self._set_config_field_if_idle(self.ff100_us_var, kv["us"], "100% us"):
                    changed.append("100% us")
            self._try_finalize_ff_model_pull()
        elif kind == "FFMODEL":
            rx = ensure_model_rx()
            # Model metadata begins a new model transaction. Preserve endpoint
            # RPM/PWM frames already received in the same GETCFG response.
            rx["coefficients"] = {}
            rx["points"] = {}
            rx["model_type"] = self.to_int(kv.get("type"), 0)
            rx["point_count"] = self.to_int(kv.get("points"), 0)
            rx["order"] = self.to_int(kv.get("order"), 1)
            self._try_finalize_ff_model_pull()
        elif kind == "FFCOEFF":
            rx = ensure_model_rx()
            index = self.to_int(kv.get("index"), -1)
            if 0 <= index <= 3 and "value" in kv:
                coeffs = rx.setdefault("coefficients", {})
                assert isinstance(coeffs, dict)
                coeffs[index] = self.to_float(kv.get("value"), 0.0)
            self._try_finalize_ff_model_pull()
        elif kind == "FFPOINT":
            rx = ensure_model_rx()
            index = self.to_int(kv.get("index"), -1)
            if 0 <= index < 12 and "pct" in kv and "us" in kv:
                points = rx.setdefault("points", {})
                assert isinstance(points, dict)
                points[index] = (self.to_float(kv.get("pct"), 0.0), self.to_float(kv.get("us"), 0.0))
            self._try_finalize_ff_model_pull()
        elif kind == "MISC":
            if "start_us" in kv and self._set_config_field_if_idle(self.start_us_var, kv["start_us"], "Start us"):
                changed.append("Start us")
            if "hold_ms" in kv and self._set_config_field_if_idle(self.start_hold_ms_var, kv["hold_ms"], "Start hold ms"):
                changed.append("Start hold")
            if "high_raw" in kv and self._set_config_field_if_idle(self.hall_high_raw_var, kv["high_raw"], "RPM high raw"):
                changed.append("RPM high")
            if "low_raw" in kv and self._set_config_field_if_idle(self.hall_low_raw_var, kv["low_raw"], "RPM low raw"):
                changed.append("RPM low")
        else:
            return

        if changed:
            self.tuning_status.set("Pulled selected-motor config: " + ", ".join(changed))
            self.save_settings(silent=True)

    def send_config_field(self, key: str) -> None:
        if key in {"kp", "ki", "kd", "limit"}:
            self.send_pid_config_only()
        elif key == "ff0_us":
            self.send_ff0_config_only()
        elif key in {"ff0_rpm", "ff100_rpm", "ff100_us"}:
            self.open_calibration_window()
            self.tuning_status.set(
                "Target-RPM and full-model edits are handled in the calibration window. The main-page 0% PWM field sends a quick live endpoint adjustment."
            )
        elif key in {"start_us", "start_hold_ms"}:
            self.apply_start_config_only()
        elif key in {"hall_high", "hall_low"}:
            self.apply_rpm_thresholds()

    def send_pid_config_only(self) -> None:
        if not self.reselect_selected_board():
            return
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            limit = float(self.limit_var.get())
        except ValueError:
            messagebox.showerror("Bad PID value", "Kp, Ki, Kd, and PID limit must be numeric.")
            return
        if min(kp, ki, kd) < 0.0:
            messagebox.showerror("Bad PID", "PID gains must be non-negative.")
            return
        if limit <= 0.0 or limit > 2000.0:
            messagebox.showerror("Bad PID limit", "PID correction limit must be in (0, 2000] us.")
            return
        self._send_reliable_config(
            f"PID {kp:.8g} {ki:.8g} {kd:.8g} {limit:.8g}",
            [(self.kp_var, kp, "Kp"), (self.ki_var, ki, "Ki"), (self.kd_var, kd, "Kd"), (self.limit_var, limit, "PID limit")],
            f"PID sent to selected motor: Kp={kp:.8g}, Ki={ki:.8g}, Kd={kd:.8g}, limit={limit:.8g} us.",
        )
        self.save_settings(silent=True)

    def send_ff0_config_only(self) -> None:
        if not self.reselect_selected_board():
            return
        assert self.selected_board_id is not None
        rpm = self.ff0_rpm_confirmed_by_board.get(self.selected_board_id)
        if rpm is None:
            messagebox.showerror(
                "Pull selected engine first",
                "Pull all configs from the selected motor once before using the quick 0% PWM button. "
                "This prevents an edited or stale RPM field from being sent accidentally.",
            )
            return
        try:
            us = int(float(self.ff0_us_var.get()))
        except ValueError:
            messagebox.showerror("Bad 0% feedforward", "0% throttle PWM must be numeric.")
            return
        if rpm < 0.0 or not (1000 <= us <= 2000):
            messagebox.showerror("Bad 0% feedforward", "0% RPM must be non-negative and 0% us must be 1000..2000.")
            return
        if not self._send_reliable_config(
            f"FF0 {rpm:.8g} {us}",
            [(self.ff0_us_var, float(us), "0% us")],
            f"Quick live 0% endpoint sent to selected motor: exact endpoint {rpm:.1f} RPM, {us} us.",
        ):
            return
        self.ff_model_pulled_board_id = None
        self.ff_model_pulled_snapshot = None
        self.ff_fit_cache = None
        self.ff_model_summary_var.set("Engine 0% endpoint changed from the main page — pull this engine again before model editing or commit.")
        self.save_settings(silent=True)

    def send_ff100_config_only(self) -> None:
        if not self.reselect_selected_board():
            return
        try:
            rpm = float(self.ff100_rpm_var.get())
            us = int(float(self.ff100_us_var.get()))
        except ValueError:
            messagebox.showerror("Bad 100% feedforward", "100% RPM and 100% us must be numeric.")
            return
        if rpm <= 0.0 or not (1000 <= us <= 2000):
            messagebox.showerror("Bad 100% feedforward", "100% RPM must be positive and 100% us must be 1000..2000.")
            return
        self.send_line(f"FF100 {rpm:.8g} {us}")
        self.tuning_status.set(f"100% feedforward sent to selected motor: {rpm:.1f} RPM, {us} us")
        self.save_settings(silent=True)

    def apply_tuning(self) -> None:
        """Apply only non-feedforward tuning values.

        Feedforward model/RPM edits are intentionally excluded because the
        calibration window uses a staged atomic commit for one selected engine.
        """
        if not self.reselect_selected_board():
            return
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
            limit = float(self.limit_var.get())
            start_us = int(float(self.start_us_var.get()))
            start_hold_ms = int(float(self.start_hold_ms_var.get()))
            hall_high_raw = int(float(self.hall_high_raw_var.get()))
            hall_low_raw = int(float(self.hall_low_raw_var.get()))
        except ValueError:
            messagebox.showerror("Bad tuning value", "PID, startup, and RPM threshold fields must be numeric.")
            return

        if min(kp, ki, kd) < 0.0:
            messagebox.showerror("Bad PID", "PID gains must be non-negative.")
            return
        if limit <= 0.0 or limit > 2000.0:
            messagebox.showerror("Bad PID limit", "PID correction limit must be in (0, 2000] us.")
            return
        if not 1000 <= start_us <= 2000:
            messagebox.showerror("Bad start PWM", "Start throttle must stay inside 1000..2000 us.")
            return
        if not 0 <= start_hold_ms <= 60000:
            messagebox.showerror("Bad start hold", "Start hold time must be 0..60000 ms.")
            return
        if not self._validate_rpm_thresholds(hall_high_raw, hall_low_raw):
            return

        self._send_reliable_config(
            f"PID {kp:.8g} {ki:.8g} {kd:.8g} {limit:.8g}",
            [(self.kp_var, kp, "Kp"), (self.ki_var, ki, "Ki"), (self.kd_var, kd, "Kd"), (self.limit_var, limit, "PID limit")],
            "PID command sent.",
        )
        self._send_reliable_config(
            f"STARTCFG {start_us} {start_hold_ms}",
            [(self.start_us_var, float(start_us), "Start us"), (self.start_hold_ms_var, float(start_hold_ms), "Start hold ms")],
            "Startup command sent.",
        )
        self._send_reliable_config(
            f"THRESH {hall_high_raw} {hall_low_raw}",
            [(self.hall_high_raw_var, float(hall_high_raw), "RPM high raw"), (self.hall_low_raw_var, float(hall_low_raw), "RPM low raw")],
            "RPM threshold command sent.",
        )
        self.tuning_status.set(
            "PID + startup + RPM thresholds sent with automatic retries and pull-back confirmation. "
            "Feedforward changes remain local until calibration-window SEND + LIVE COMMIT."
        )
        self.save_settings(silent=True)

    def _validate_rpm_thresholds(self, high_raw: int, low_raw: int) -> bool:
        if not (0 <= low_raw <= 4095 and 0 <= high_raw <= 4095 and high_raw > low_raw):
            messagebox.showerror("Bad RPM sensor thresholds", "Use ADC raw counts 0..4095, with high threshold greater than low threshold.")
            return False
        return True

    def apply_rpm_thresholds(self) -> None:
        if not self.reselect_selected_board():
            return
        try:
            high_raw = int(float(self.hall_high_raw_var.get()))
            low_raw = int(float(self.hall_low_raw_var.get()))
        except ValueError:
            messagebox.showerror("Bad RPM sensor thresholds", "Threshold fields must be numeric raw ADC counts.")
            return
        if not self._validate_rpm_thresholds(high_raw, low_raw):
            return
        self._send_reliable_config(
            f"THRESH {high_raw} {low_raw}",
            [(self.hall_high_raw_var, float(high_raw), "RPM high raw"), (self.hall_low_raw_var, float(low_raw), "RPM low raw")],
            f"RPM sensor thresholds sent: high={high_raw}, low={low_raw}.",
        )
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
            self._set_config_field_if_idle(var, str(value), "/".join(path))

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
        set_string(self.limit_var, ("pid", "limit_us"), "1000")
        set_string(self.ff0_rpm_var, ("feedforward", "rpm_0_pct"), "2200")
        set_string(self.ff0_us_var, ("feedforward", "us_0_pct"), "1850")
        set_string(self.ff100_rpm_var, ("feedforward", "rpm_100_pct"), "4250")
        set_string(self.ff100_us_var, ("feedforward", "us_100_pct"), "1450")
        set_string(self.start_us_var, ("startup", "start_us"), "1400")
        set_string(self.start_hold_ms_var, ("startup", "hold_after_rpm_ms"), "1000")
        set_string(self.manual_pwm_hold_ms_var, ("manual_pwm_test", "hold_ms"), "10000")
        set_string(self.hall_high_raw_var, ("rpm_sensor", "threshold_high_raw"), "215")
        set_string(self.hall_low_raw_var, ("rpm_sensor", "threshold_low_raw"), "83")
        set_string(self.hall_cal_target_rpm_var, ("rpm_sensor", "auto_cal_target_rpm"), "2200")
        set_string(self.hall_cal_duration_var, ("rpm_sensor", "auto_cal_duration_s"), "0")
        set_string(self.endpoint_auto_rate_var, ("endpoint_auto_adjust", "rate_us_per_s"), "25")
        set_string(self.endpoint_auto_duration_var, ("endpoint_auto_adjust", "all_board_dwell_s"), "15")
        set_string(self.endpoint_auto_start_us_var, ("endpoint_auto_adjust", "start_us"), "1700")
        set_string(self.sweep_start_var, ("sweep", "start_pct"), "0")
        set_string(self.sweep_end_var, ("sweep", "end_pct"), "100")
        set_string(self.sweep_duration_var, ("sweep", "duration_s"), "12")
        set_string(self.square_min_var, ("square_wave", "minimum_pct"), "15")
        set_string(self.square_max_var, ("square_wave", "maximum_pct"), "35")
        set_string(self.square_period_var, ("square_wave", "period_s"), "4")
        set_string(self.scope_trigger_rpm_var, ("rpm_scope", "trigger_rpm"), "3000")
        set_string(self.scope_width_s_var, ("rpm_scope", "width_s"), "4")
        set_string(self.telemetry_a_ms_var, ("telemetry_periods_ms", "a"), "50")
        set_string(self.telemetry_b_ms_var, ("telemetry_periods_ms", "b"), "100")
        set_string(self.telemetry_c_ms_var, ("telemetry_periods_ms", "c"), "250")
        set_string(self.ff_model_type_var, ("feedforward_calibration", "model_type"), "Piecewise linear")
        set_string(self.ff_poly_order_var, ("feedforward_calibration", "polynomial_order"), "2")
        set_string(self.ff_model_point_count_var, ("feedforward_calibration", "point_count"), "7")
        set_string(self.ff_cal_start_us_var, ("feedforward_calibration", "start_us"), "1700")
        set_string(self.ff_cal_rate_var, ("feedforward_calibration", "adjust_rate_us_per_s"), "25")
        set_string(self.ff_cal_deadband_var, ("feedforward_calibration", "rpm_deadband"), "50")
        set_string(self.ff_cal_stable_s_var, ("feedforward_calibration", "stable_time_s"), "2")
        set_string(self.ff_intermediate_min_var, ("feedforward_calibration", "intermediate_min_pct"), "10")
        set_string(self.ff_intermediate_max_var, ("feedforward_calibration", "intermediate_max_pct"), "90")
        set_string(self.ff_point_time_var, ("feedforward_calibration", "sample_time_per_point_s"), "5")
        set_string(self.ff_point_timeout_var, ("feedforward_calibration", "point_timeout_s"), "45")
        set_string(self.ff_sweep_total_s_var, ("feedforward_calibration", "sweep_total_s"), "30")
        set_string(self.ff_sweep_start_us_var, ("feedforward_calibration", "sweep_start_us"), "1850")
        set_string(self.ff_sweep_end_us_var, ("feedforward_calibration", "sweep_end_us"), "1450")
        set_string(self.ff_sweep_endpoint_dwell_s_var, ("feedforward_calibration", "sweep_endpoint_hold_s"), "2")
        try:
            self.ff_use_sweep_var.set(bool(self._nested_get(data, ("feedforward_calibration", "use_sweep_points"), True)))
        except tk.TclError:
            self.ff_use_sweep_var.set(True)
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
            "startup": {
                "start_us": self.start_us_var.get(),
                "hold_after_rpm_ms": self.start_hold_ms_var.get(),
            },
            "manual_pwm_test": {
                "hold_ms": self.manual_pwm_hold_ms_var.get(),
            },
            "rpm_sensor": {
                "threshold_high_raw": self.hall_high_raw_var.get(),
                "threshold_low_raw": self.hall_low_raw_var.get(),
                "auto_cal_target_rpm": self.hall_cal_target_rpm_var.get(),
                "auto_cal_duration_s": "0",
            },
            "endpoint_auto_adjust": {
                "rate_us_per_s": self.endpoint_auto_rate_var.get(),
                "all_board_dwell_s": self.endpoint_auto_duration_var.get(),
                "start_us": self.endpoint_auto_start_us_var.get(),
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
            "rpm_scope": {
                "trigger_rpm": self.scope_trigger_rpm_var.get(),
                "width_s": self.scope_width_s_var.get(),
            },
            "feedforward_calibration": {
                "model_type": self.ff_model_type_var.get(),
                "polynomial_order": self.ff_poly_order_var.get(),
                "point_count": self.ff_model_point_count_var.get(),
                "start_us": self.ff_cal_start_us_var.get(),
                "adjust_rate_us_per_s": self.ff_cal_rate_var.get(),
                "rpm_deadband": self.ff_cal_deadband_var.get(),
                "stable_time_s": self.ff_cal_stable_s_var.get(),
                "intermediate_min_pct": self.ff_intermediate_min_var.get(),
                "intermediate_max_pct": self.ff_intermediate_max_var.get(),
                "sample_time_per_point_s": self.ff_point_time_var.get(),
                "point_timeout_s": self.ff_point_timeout_var.get(),
                "sweep_total_s": self.ff_sweep_total_s_var.get(),
                "sweep_start_us": self.ff_sweep_start_us_var.get(),
                "sweep_end_us": self.ff_sweep_end_us_var.get(),
                "sweep_endpoint_hold_s": self.ff_sweep_endpoint_dwell_s_var.get(),
                "use_sweep_points": bool(self.ff_use_sweep_var.get()),
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

    # ---------------- Feedforward calibration + fitting window ----------------
    def open_calibration_window(self) -> None:
        if self.calibration_window is not None and self.calibration_window.winfo_exists():
            self.calibration_window.lift()
            self.update_calibration_plot()
            return

        win = tk.Toplevel(self)
        win.title("Selected-engine feedforward calibration and model fitting")
        win.geometry("1220x900")
        win.minsize(1050, 760)
        win.protocol("WM_DELETE_WINDOW", self.close_calibration_window)
        self.calibration_window = win

        frame = ttk.Frame(win, padding=10)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(4, weight=1)

        safety = ttk.LabelFrame(frame, text="Selected-engine model transfer — explicit pull and atomic live commit", padding=8)
        safety.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for col in range(6):
            safety.columnconfigure(col, weight=1)
        ttk.Button(safety, text="Pull model from selected engine", command=self.pull_feedforward_model).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(safety, text="Fit / preview local points", command=self.fit_feedforward_model).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(safety, text="SEND + LIVE COMMIT to selected engine", command=self.send_feedforward_model).grid(row=0, column=2, columnspan=2, sticky="ew", padx=4)
        ttk.Button(safety, text="Reset local values", command=self.reset_local_feedforward_model).grid(row=0, column=4, sticky="ew", padx=4)
        ttk.Button(safety, text="Reset engine to linear", command=self.reset_engine_feedforward_model).grid(row=0, column=5, sticky="ew", padx=(4, 0))
        banner = tk.Label(safety, textvariable=self.ff_model_summary_var, wraplength=1130, anchor="w", justify="left", padx=6, pady=4)
        banner.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(6, 0))
        self.ff_model_banner_label = banner

        model = ttk.LabelFrame(frame, text="Model and fitting parameters", padding=8)
        model.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        for col in range(8):
            model.columnconfigure(col, weight=1)
        ttk.Label(model, text="Model").grid(row=0, column=0, sticky="w")
        combo = ttk.Combobox(model, textvariable=self.ff_model_type_var, state="readonly", values=("Linear", "Polynomial", "Piecewise linear"))
        combo.grid(row=1, column=0, sticky="ew", padx=(0, 5))
        ttk.Label(model, text="Polynomial order").grid(row=0, column=1, sticky="w")
        order = ttk.Combobox(model, textvariable=self.ff_poly_order_var, state="readonly", values=("1", "2", "3"), width=6)
        order.grid(row=1, column=1, sticky="ew", padx=5)
        ttk.Label(model, text="Model / intermediate points").grid(row=0, column=2, sticky="w")
        point_count_entry = tk.Entry(model, textvariable=self.ff_model_point_count_var, width=8)
        point_count_entry.grid(row=1, column=2, sticky="ew", padx=5)
        ttk.Label(model, text="0% target RPM").grid(row=0, column=3, sticky="w")
        idle_rpm_entry = tk.Entry(model, textvariable=self.ff0_rpm_var, width=10)
        idle_rpm_entry.grid(row=1, column=3, sticky="ew", padx=5)
        ttk.Label(model, text="100% target RPM").grid(row=0, column=4, sticky="w")
        max_rpm_entry = tk.Entry(model, textvariable=self.ff100_rpm_var, width=10)
        max_rpm_entry.grid(row=1, column=4, sticky="ew", padx=5)
        ttk.Checkbutton(model, text="Use sweep points in fit", variable=self.ff_use_sweep_var, command=self._mark_ff_model_dirty).grid(row=1, column=5, sticky="w", padx=5)
        ttk.Button(model, text="Save points CSV", command=self.save_calibration_csv).grid(row=1, column=6, sticky="ew", padx=5)
        ttk.Button(model, text="Load points CSV", command=self.load_calibration_csv).grid(row=1, column=7, sticky="ew", padx=(5, 0))

        self.ff_changed_widgets = [point_count_entry, idle_rpm_entry, max_rpm_entry]
        if not self.ff_traces_installed:
            for var in (self.ff_model_type_var, self.ff_poly_order_var, self.ff_model_point_count_var, self.ff0_rpm_var, self.ff100_rpm_var):
                var.trace_add("write", lambda *_args: self._mark_ff_model_dirty())
            self.ff_traces_installed = True
        combo.bind("<<ComboboxSelected>>", lambda _e: self._mark_ff_model_dirty())
        order.bind("<<ComboboxSelected>>", lambda _e: self._mark_ff_model_dirty())

        actions = ttk.LabelFrame(frame, text="Calibration actions — selected engine only; calibration never writes the model", padding=8)
        actions.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        for col in range(10):
            actions.columnconfigure(col, weight=1)

        action_fields = (
            ("Start PWM us", self.ff_cal_start_us_var),
            ("Max adjust us/s", self.ff_cal_rate_var),
            ("RPM deadband", self.ff_cal_deadband_var),
            ("Stable time s", self.ff_cal_stable_s_var),
            ("Intermediate min %", self.ff_intermediate_min_var),
            ("Intermediate max %", self.ff_intermediate_max_var),
            ("Sample time / point s", self.ff_point_time_var),
            ("Max search / point s", self.ff_point_timeout_var),
            ("Sweep endpoint hold s", self.ff_sweep_endpoint_dwell_s_var),
        )
        for i, (label, var) in enumerate(action_fields):
            ttk.Label(actions, text=label).grid(row=0, column=i, sticky="w")
            ttk.Entry(actions, textvariable=var, width=9).grid(row=1, column=i, sticky="ew", padx=(0 if i == 0 else 3, 3))

        ttk.Button(actions, text="Start 0% calibration (runs until stopped)", command=lambda: self.start_ff_endpoint_calibration(0.0)).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(7, 0), padx=(0, 3))
        ttk.Button(actions, text="Start 100% calibration (runs until stopped)", command=lambda: self.start_ff_endpoint_calibration(100.0)).grid(row=2, column=2, columnspan=2, sticky="ew", pady=(7, 0), padx=3)
        ttk.Button(actions, text="Capture current endpoint + stop", command=lambda: self.stop_feedforward_calibration(capture=True)).grid(row=2, column=4, columnspan=2, sticky="ew", pady=(7, 0), padx=3)
        ttk.Button(actions, text="Auto-calibrate intermediate points", command=self.start_ff_intermediate_calibration).grid(row=2, column=6, columnspan=2, sticky="ew", pady=(7, 0), padx=3)
        ttk.Button(actions, text="ABORT calibration → 0%", command=lambda: self.stop_feedforward_calibration(capture=False)).grid(row=2, column=8, columnspan=2, sticky="ew", pady=(7, 0), padx=(3, 0))

        ttk.Label(actions, text="0→100% PWM ramp time s (x-axis is measured RPM normalized to exact target RPM endpoints)").grid(row=3, column=0, columnspan=3, sticky="w", pady=(7, 0))
        ttk.Entry(actions, textvariable=self.ff_sweep_total_s_var, width=9).grid(row=4, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(actions, text="Start RPM-mapped PWM sweep + plot curve", command=self.start_ff_pwm_sweep).grid(row=4, column=1, columnspan=3, sticky="ew", padx=3)
        ttk.Button(actions, text="Erase all graph points", command=self.clear_calibration_samples).grid(row=4, column=4, columnspan=2, sticky="ew", padx=3)
        ttk.Button(actions, text="Delete selected point", command=self.delete_selected_calibration_point).grid(row=4, column=6, columnspan=2, sticky="ew", padx=3)
        ttk.Button(actions, text="Toggle selected point use", command=self.toggle_selected_calibration_point).grid(row=4, column=8, columnspan=2, sticky="ew", padx=(3, 0))

        ttk.Label(frame, textvariable=self.calibration_status, wraplength=1160).grid(row=3, column=0, sticky="ew", pady=(0, 6))

        lower = ttk.Panedwindow(frame, orient="horizontal")
        lower.grid(row=4, column=0, sticky="nsew")

        graph_frame = ttk.Frame(lower)
        table_frame = ttk.Frame(lower)
        lower.add(graph_frame, weight=3)
        lower.add(table_frame, weight=2)
        graph_frame.rowconfigure(0, weight=1)
        graph_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

        self.calibration_figure = Figure(figsize=(7.6, 5.2), dpi=100)
        self.calibration_axes = self.calibration_figure.add_subplot(111)
        self.calibration_rpm_axes = self.calibration_axes.twinx()
        self.calibration_canvas = FigureCanvasTkAgg(self.calibration_figure, master=graph_frame)
        self.calibration_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        columns = ("use", "pct", "rpm", "us", "source")
        tree = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="browse", height=18)
        for col, title, width in (("use", "Fit", 45), ("pct", "Throttle %", 85), ("rpm", "RPM (exact at endpoints)", 130), ("us", "PWM us", 75), ("source", "Source", 115)):
            tree.heading(col, text=title)
            tree.column(col, width=width, anchor="center")
        tree.grid(row=0, column=0, columnspan=4, sticky="nsew")
        scroll = ttk.Scrollbar(table_frame, orient="vertical", command=tree.yview)
        scroll.grid(row=0, column=4, sticky="ns")
        tree.configure(yscrollcommand=scroll.set)
        tree.bind("<<TreeviewSelect>>", self.on_calibration_point_selected)
        self.ff_point_tree = tree

        ttk.Label(table_frame, text="Throttle %").grid(row=1, column=0, sticky="w", pady=(7, 0))
        ttk.Label(table_frame, text="Measured RPM (0/100 forced exact)").grid(row=1, column=1, sticky="w", pady=(7, 0))
        ttk.Label(table_frame, text="PWM us").grid(row=1, column=2, sticky="w", pady=(7, 0))
        ttk.Entry(table_frame, textvariable=self.ff_point_edit_pct_var, width=9).grid(row=2, column=0, sticky="ew", padx=(0, 3))
        ttk.Entry(table_frame, textvariable=self.ff_point_edit_rpm_var, width=9).grid(row=2, column=1, sticky="ew", padx=3)
        ttk.Entry(table_frame, textvariable=self.ff_point_edit_us_var, width=9).grid(row=2, column=2, sticky="ew", padx=3)
        ttk.Checkbutton(table_frame, text="Use", variable=self.ff_point_edit_use_var).grid(row=2, column=3, sticky="w", padx=3)
        ttk.Button(table_frame, text="Update selected point", command=self.update_selected_calibration_point).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(7, 0), padx=(0, 3))
        ttk.Button(table_frame, text="Add as new manual point", command=self.add_manual_calibration_point).grid(row=3, column=2, columnspan=2, sticky="ew", pady=(7, 0), padx=(3, 0))
        ttk.Button(table_frame, text="Enable all points for fit", command=lambda: self.set_all_calibration_points_enabled(True)).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0), padx=(0, 3))
        ttk.Button(table_frame, text="Exclude all points", command=lambda: self.set_all_calibration_points_enabled(False)).grid(row=4, column=2, columnspan=2, sticky="ew", pady=(6, 0), padx=(3, 0))

        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.calibration_dirty = True
        self.update_calibration_plot()

    def close_calibration_window(self) -> None:
        if self.ff_cal_mode != "idle":
            self.stop_feedforward_calibration(capture=False, silent=True)
        if self.calibration_window is not None and self.calibration_window.winfo_exists():
            self.calibration_window.destroy()
        self.calibration_window = None
        self.calibration_figure = None
        self.calibration_axes = None
        self.calibration_rpm_axes = None
        self.calibration_canvas = None
        self.ff_point_tree = None
        self.ff_model_banner_label = None

    def _mark_ff_model_dirty(self, *_args: object) -> None:
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.calibration_dirty = True
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()

    def _refresh_ff_dirty_highlight(self) -> None:
        bg = "#fff0b3" if self.ff_model_dirty else "white"
        for widget in self.ff_changed_widgets:
            try:
                widget.configure(background=bg)
            except tk.TclError:
                pass
        if self.ff_model_banner_label is not None and self.ff_model_banner_label.winfo_exists():
            self.ff_model_banner_label.configure(background="#fff0b3" if self.ff_model_dirty else "#dff4df")
        board = f"board {self.ff_model_pulled_board_id}" if self.ff_model_pulled_board_id is not None else "no board"
        if self.ff_model_dirty:
            self.ff_model_summary_var.set(f"UNSENT LOCAL CHANGES — last pull: {board}. Fit and commit are required to change the engine.")
        elif self.ff_model_pulled_board_id is not None:
            self.ff_model_summary_var.set(f"Pulled model matches local fields for board {self.ff_model_pulled_board_id}; no unsent edits.")

    def pull_feedforward_model(self) -> None:
        if not self.reselect_selected_board():
            return
        if self.ff_model_dirty and not messagebox.askyesno("Discard local model edits?", "Pulling will replace the local model fields and graph points. Continue?"):
            return
        assert self.selected_board_id is not None
        self._begin_explicit_config_pull()
        self.ff_model_pull_requested = True
        self.ff_model_receive = {"coefficients": {}, "points": {}, "board_id": self.selected_board_id}
        self.ff_model_pulled_board_id = None
        self.ff_pending_commit_snapshot = None
        self.send_line("FFCANCEL")  # clears any incomplete staged transfer; active model is untouched
        self.send_line("GETCFG")
        self.calibration_status.set(f"Requested model from selected board {self.selected_board_id}; waiting for complete CFG model frames")

    def _try_finalize_ff_model_pull(self) -> None:
        rx = self.ff_model_receive
        required = ("board_id", "model_type", "point_count", "order", "idle_rpm", "max_rpm")
        if any(key not in rx for key in required):
            return
        board_id = int(rx["board_id"])
        if self.selected_board_id != board_id:
            return
        model_type = int(rx["model_type"])
        point_count = int(rx["point_count"])
        order = int(rx["order"])
        coeffs = rx.get("coefficients", {})
        points = rx.get("points", {})
        if model_type == 1:
            if any(i not in coeffs for i in range(order + 1)):
                return
        else:
            if point_count < 2 or any(i not in points for i in range(point_count)):
                return

        # Ordinary config confirmation requests update the main-page fields only.
        # They must not erase local calibration points or replace a fit preview.
        if not self.ff_model_pull_requested and self.ff_pending_commit_snapshot is None:
            return

        type_name = {0: "Linear", 1: "Polynomial", 2: "Piecewise linear"}.get(model_type, "Linear")
        self.ff_model_type_var.set(type_name)
        self.ff_poly_order_var.set(str(max(1, min(order, 3))))
        self.ff_model_point_count_var.set(str(max(2, point_count if point_count else 7)))
        self._set_config_field_if_idle(self.ff0_rpm_var, f"{float(rx['idle_rpm']):.6g}", "0% RPM")
        self._set_config_field_if_idle(self.ff100_rpm_var, f"{float(rx['max_rpm']):.6g}", "100% RPM")
        if "idle_us" in rx:
            self._set_config_field_if_idle(self.ff0_us_var, str(int(float(rx["idle_us"]))), "0% us")
        if "max_us" in rx:
            self._set_config_field_if_idle(self.ff100_us_var, str(int(float(rx["max_us"]))), "100% us")

        self.ff_calibration_points.clear()
        idle_rpm = float(rx["idle_rpm"])
        max_rpm = float(rx["max_rpm"])
        if model_type == 1:
            local_coeffs = [float(coeffs.get(i, 0.0)) for i in range(4)]
            sample_count = max(7, int(self.ff_model_point_count_var.get()))
            for pct in np.linspace(0.0, 100.0, sample_count):
                x = pct / 100.0
                us = sum(local_coeffs[i] * (x ** i) for i in range(order + 1))
                rpm = idle_rpm + x * (max_rpm - idle_rpm)
                self.ff_calibration_points.append(FeedforwardCalibrationPoint(float(pct), rpm, float(us), "pulled model", False))
            fit = {"model_type": 1, "order": order, "point_count": 0, "coefficients": local_coeffs, "points": []}
        else:
            model_points: list[tuple[float, float]] = []
            for i in range(point_count):
                pct, us = points[i]
                rpm = idle_rpm + (pct / 100.0) * (max_rpm - idle_rpm)
                self.ff_calibration_points.append(FeedforwardCalibrationPoint(float(pct), rpm, float(us), "pulled model", False))
                model_points.append((float(pct), float(us)))
            fit = {"model_type": model_type, "order": 1, "point_count": point_count, "coefficients": [0.0] * 4, "points": model_points}

        fit["idle_rpm"] = idle_rpm
        fit["max_rpm"] = max_rpm
        received_snapshot = {
            "board_id": board_id,
            "model_type": model_type,
            "order": order,
            "idle_rpm": idle_rpm,
            "max_rpm": max_rpm,
            "coefficients": [float(coeffs.get(i, 0.0)) for i in range(4)],
            "points": list(fit.get("points", [])),
        }
        pending = self.ff_pending_commit_snapshot
        verified = pending is not None and self._feedforward_snapshot_matches(pending, received_snapshot)
        mismatch = pending is not None and not verified
        self.ff_fit_cache = fit
        self.ff_model_pulled_snapshot = received_snapshot
        self.ff_model_pulled_board_id = board_id
        self.ff_model_dirty = False
        self.ff_model_pull_requested = False
        self.ff_pending_commit_snapshot = None
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        if verified:
            self.calibration_status.set(f"COMMIT VERIFIED on board {board_id}: pull-back matches the RPM endpoints and fitted model sent")
        elif mismatch:
            self.calibration_status.set(f"WARNING: board {board_id} pull-back does not match the staged model. The controller likely rejected it; the graph now shows the model that remains active.")
        else:
            self.calibration_status.set(f"Pulled complete feedforward model from board {board_id}; pulled reference points are excluded from fitting until explicitly enabled")
        self.calibration_dirty = True
        self.update_calibration_plot()

    @staticmethod
    def _feedforward_snapshot_matches(expected: dict[str, object], actual: dict[str, object]) -> bool:
        try:
            if int(expected["board_id"]) != int(actual["board_id"]):
                return False
            if int(expected["model_type"]) != int(actual["model_type"]):
                return False
            if abs(float(expected.get("idle_rpm", actual["idle_rpm"])) - float(actual["idle_rpm"])) > 0.5:
                return False
            if abs(float(expected.get("max_rpm", actual["max_rpm"])) - float(actual["max_rpm"])) > 0.5:
                return False
            if expected.get("reset_only"):
                return int(actual["model_type"]) == 0
            if int(expected.get("order", 1)) != int(actual.get("order", 1)):
                return False
            model_type = int(expected["model_type"])
            if model_type == 1:
                expected_coeffs = list(expected.get("coefficients", []))
                actual_coeffs = list(actual.get("coefficients", []))
                if len(actual_coeffs) < len(expected_coeffs):
                    return False
                return all(abs(float(a) - float(b)) <= 0.02 for a, b in zip(expected_coeffs, actual_coeffs))
            expected_points = list(expected.get("points", []))
            actual_points = list(actual.get("points", []))
            if len(expected_points) != len(actual_points):
                return False
            return all(
                abs(float(ep[0]) - float(ap[0])) <= 0.011 and abs(float(ep[1]) - float(ap[1])) <= 0.51
                for ep, ap in zip(expected_points, actual_points)
            )
        except (KeyError, TypeError, ValueError):
            return False

    def _calibration_preflight(self) -> Optional[int]:
        if not self.serial_worker.is_connected():
            messagebox.showerror("Bridge disconnected", "Connect the USB↔CAN bridge first.")
            return None
        if not self.reselect_selected_board():
            return None
        if not self.command_armed:
            messagebox.showerror("Engine not armed", "Arm the selected engine before direct-PWM calibration. Calibration will not arm it automatically.")
            return None
        assert self.selected_board_id is not None
        if self.ff_model_pulled_board_id != self.selected_board_id:
            messagebox.showerror("Pull required", "Pull the feedforward model from this selected engine before calibrating or sending. This prevents editing the wrong engine.")
            return None
        if self.ff_cal_mode != "idle":
            messagebox.showerror("Calibration active", "Stop the current feedforward calibration first.")
            return None
        return self.selected_board_id

    def _read_ff_calibration_settings(self) -> Optional[dict[str, float]]:
        try:
            values = {
                "start_us": float(self.ff_cal_start_us_var.get()),
                "rate": float(self.ff_cal_rate_var.get()),
                "deadband": float(self.ff_cal_deadband_var.get()),
                "stable_s": float(self.ff_cal_stable_s_var.get()),
                "sample_s": float(self.ff_point_time_var.get()),
                "timeout_s": float(self.ff_point_timeout_var.get()),
            }
        except ValueError:
            messagebox.showerror("Bad calibration setting", "Calibration PWM, rate, RPM deadband, and times must be numeric.")
            return None
        if not 1000.0 <= values["start_us"] <= 2000.0:
            messagebox.showerror("Bad start PWM", "Calibration start PWM must be 1000..2000 us.")
            return None
        if not 0.1 <= values["rate"] <= 250.0 or not 1.0 <= values["deadband"] <= 2000.0:
            messagebox.showerror("Bad search settings", "Max adjust rate must be 0.1..250 us/s and deadband must be 1..2000 RPM.")
            return None
        if values["stable_s"] < 0.0 or values["sample_s"] <= 0.0 or values["timeout_s"] <= values["sample_s"]:
            messagebox.showerror("Bad timing", "Stable time must be non-negative; sample time must be positive; timeout must exceed sample time.")
            return None
        return values

    def start_ff_endpoint_calibration(self, throttle_pct: float) -> None:
        board_id = self._calibration_preflight()
        settings = self._read_ff_calibration_settings()
        if board_id is None or settings is None:
            return
        try:
            idle_rpm = float(self.ff0_rpm_var.get())
            max_rpm = float(self.ff100_rpm_var.get())
        except ValueError:
            messagebox.showerror("Bad RPM endpoints", "0% and 100% target RPM must be numeric.")
            return
        self.ff_cal_mode = "endpoint"
        self.ff_cal_board_id = board_id
        self.ff_cal_target_pct = throttle_pct
        self.ff_cal_target_rpm = idle_rpm if throttle_pct <= 0.0 else max_rpm
        self.ff_cal_current_us = settings["start_us"]
        self.ff_cal_last_update = time.monotonic()
        self.ff_cal_stable_since = None
        self.ff_cal_sample_started = None
        self.ff_cal_recent_rpm.clear()
        self.ff_cal_recent_us.clear()
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.manual_pwm_bypass_var.set(True)
        self.manual_pwm_slider_us_var.set(self.ff_cal_current_us)
        self.send_current_command()
        self.send_pwm_bypass_us(int(round(self.ff_cal_current_us)), live=False)
        self.calibration_status.set(f"Board {board_id}: {throttle_pct:.0f}% calibration running indefinitely toward {self.ff_cal_target_rpm:.0f} RPM. Press Capture current endpoint + stop when stable.")

    def start_ff_intermediate_calibration(self) -> None:
        board_id = self._calibration_preflight()
        settings = self._read_ff_calibration_settings()
        if board_id is None or settings is None:
            return
        try:
            count = int(float(self.ff_model_point_count_var.get()))
            minimum = float(self.ff_intermediate_min_var.get())
            maximum = float(self.ff_intermediate_max_var.get())
        except ValueError:
            messagebox.showerror("Bad intermediate settings", "Point count and intermediate percentage range must be numeric.")
            return
        if not 1 <= count <= 12 or not 0.0 < minimum < maximum < 100.0:
            messagebox.showerror("Bad intermediate settings", "Use 1..12 points and a range strictly inside 0..100%.")
            return
        self.ff_cal_mode = "intermediate"
        self.ff_cal_board_id = board_id
        self.ff_cal_sequence = [float(v) for v in np.linspace(minimum, maximum, count)]
        self.ff_cal_sequence_index = 0
        self.ff_cal_current_us = settings["start_us"]
        self.ff_cal_last_update = time.monotonic()
        self.ff_cal_stable_since = None
        self.ff_cal_sample_started = None
        self.ff_cal_recent_rpm.clear()
        self.ff_cal_recent_us.clear()
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.manual_pwm_bypass_var.set(True)
        self._begin_next_ff_intermediate_point()
        self.send_current_command()
        self.send_pwm_bypass_us(int(round(self.ff_cal_current_us)), live=False)

    def _begin_next_ff_intermediate_point(self) -> None:
        if self.ff_cal_sequence_index >= len(self.ff_cal_sequence):
            completed = len(self.ff_cal_sequence)
            self.stop_feedforward_calibration(capture=False, silent=True)
            self.calibration_status.set(f"Intermediate calibration complete: captured {completed} requested point(s); review/fit locally, then commit explicitly")
            return
        pct = self.ff_cal_sequence[self.ff_cal_sequence_index]
        idle_rpm = float(self.ff0_rpm_var.get())
        max_rpm = float(self.ff100_rpm_var.get())
        self.ff_cal_target_pct = pct
        self.ff_cal_target_rpm = idle_rpm + (pct / 100.0) * (max_rpm - idle_rpm)
        self.ff_cal_stable_since = None
        self.ff_cal_sample_started = None
        self.ff_cal_recent_rpm.clear()
        self.ff_cal_recent_us.clear()
        self.ff_cal_point_deadline = time.monotonic() + float(self.ff_point_timeout_var.get())
        self.calibration_status.set(f"Board {self.ff_cal_board_id}: searching point {self.ff_cal_sequence_index + 1}/{len(self.ff_cal_sequence)} at {pct:.1f}% target = {self.ff_cal_target_rpm:.0f} RPM")

    def start_ff_pwm_sweep(self) -> None:
        board_id = self._calibration_preflight()
        if board_id is None:
            return
        try:
            # Prefer the newest enabled local endpoint captures, then fall back
            # to the endpoint values pulled from the selected engine. This lets a
            # 0%/100% calibration feed directly into the next sweep without an
            # intermediate model commit.
            local_zero = next(
                (p.pwm_us for p in reversed(self.ff_calibration_points)
                 if p.use_for_fit and not p.source.startswith("sweep") and abs(p.throttle_pct) <= 0.5),
                None,
            )
            local_hundred = next(
                (p.pwm_us for p in reversed(self.ff_calibration_points)
                 if p.use_for_fit and not p.source.startswith("sweep") and abs(p.throttle_pct - 100.0) <= 0.5),
                None,
            )
            start_us = float(local_zero if local_zero is not None else self.ff0_us_var.get())
            end_us = float(local_hundred if local_hundred is not None else self.ff100_us_var.get())
            idle_rpm = float(self.ff0_rpm_var.get())
            max_rpm = float(self.ff100_rpm_var.get())
            duration = float(self.ff_sweep_total_s_var.get())
            dwell = float(self.ff_sweep_endpoint_dwell_s_var.get())
        except ValueError:
            messagebox.showerror(
                "Bad sweep",
                "The selected engine's 0%/100% PWM endpoints, ramp time, and endpoint hold time must be numeric.",
            )
            return
        if not (1000.0 <= start_us <= 2000.0 and 1000.0 <= end_us <= 2000.0):
            messagebox.showerror("Bad sweep", "The selected engine's 0% and 100% PWM endpoints must be 1000..2000 us.")
            return
        if max_rpm <= idle_rpm:
            messagebox.showerror("Bad sweep", "100% target RPM must be greater than the exact 0% target RPM.")
            return
        if abs(end_us - start_us) < 1.0:
            messagebox.showerror("Bad sweep", "0% and 100% PWM endpoints must be different.")
            return
        if duration <= 1.0 or dwell < 0.0 or dwell > 60.0:
            messagebox.showerror("Bad sweep", "Ramp time must exceed 1 second and endpoint hold must be 0..60 seconds.")
            return

        # Remove previous sweep-only samples so a new run cannot silently combine
        # two different endpoint ranges. Hand-entered/end-point points are kept.
        self.ff_calibration_points = [p for p in self.ff_calibration_points if not p.source.startswith("sweep")]
        self.ff_fit_cache = None
        self.ff_model_dirty = True
        self.calibration_dirty = True

        now = time.monotonic()
        self.ff_cal_mode = "sweep"
        self.ff_cal_board_id = board_id
        self.ff_sweep_duration_s = duration
        self.ff_sweep_start_us = start_us
        self.ff_sweep_end_us = end_us
        self.ff_sweep_idle_rpm = idle_rpm
        self.ff_sweep_max_rpm = max_rpm
        self.ff_sweep_endpoint_dwell_s = dwell
        self.ff_sweep_phase = "pre_hold"
        self.ff_sweep_phase_started = now
        self.ff_sweep_started = now
        self.ff_sweep_command_pct = 0.0
        self.ff_sweep_last_recorded_time = 0.0
        self.ff_sweep_last_recorded_pct = None
        self.ff_cal_current_us = start_us
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.manual_pwm_bypass_var.set(True)
        self.send_current_command()
        self.send_pwm_bypass_us(int(round(start_us)), live=False)
        self.refresh_calibration_point_tree()
        self.update_calibration_plot()
        self.calibration_status.set(
            f"Board {board_id}: holding exact 0% endpoint {start_us:.0f} us for {dwell:.1f} s, "
            f"then sweeping to exact 100% endpoint {end_us:.0f} us over {duration:.1f} s"
        )

    def update_feedforward_calibration_task(self) -> None:
        if self.ff_cal_mode == "idle":
            return
        if self.selected_board_id != self.ff_cal_board_id or not self.command_armed:
            self.stop_feedforward_calibration(capture=False, silent=True)
            self.calibration_status.set("Calibration aborted because the selected board changed or the engine was disarmed")
            return
        now = time.monotonic()
        if self.ff_cal_mode == "sweep":
            if self.ff_sweep_phase == "pre_hold":
                self.ff_sweep_command_pct = 0.0
                self.ff_cal_current_us = self.ff_sweep_start_us
                remaining = max(self.ff_sweep_endpoint_dwell_s - (now - self.ff_sweep_phase_started), 0.0)
                if remaining <= 0.0:
                    self.add_calibration_point(
                        0.0,
                        self.ff_sweep_idle_rpm,
                        self.ff_sweep_start_us,
                        f"sweep endpoint (measured {self.telemetry.rpm:.1f} RPM)",
                    )
                    self.ff_sweep_phase = "ramp"
                    self.ff_sweep_started = now
                    self.ff_sweep_phase_started = now
                else:
                    self.calibration_status.set(
                        f"Board {self.ff_cal_board_id}: holding exact 0% / {self.ff_sweep_start_us:.0f} us "
                        f"({remaining:.1f} s remaining)"
                    )
            elif self.ff_sweep_phase == "ramp":
                alpha = min(max((now - self.ff_sweep_started) / self.ff_sweep_duration_s, 0.0), 1.0)
                self.ff_sweep_command_pct = alpha * 100.0
                # Interpolation is bounded by construction; it cannot overshoot
                # either selected-engine endpoint even if the servo direction is
                # reversed on a future installation.
                self.ff_cal_current_us = (
                    self.ff_sweep_start_us
                    + alpha * (self.ff_sweep_end_us - self.ff_sweep_start_us)
                )
                self.calibration_status.set(
                    f"Board {self.ff_cal_board_id}: exact endpoint sweep {self.ff_sweep_command_pct:.1f}% complete; "
                    f"command {self.ff_cal_current_us:.0f} us"
                )
                if alpha >= 1.0:
                    self.ff_sweep_command_pct = 100.0
                    self.ff_cal_current_us = self.ff_sweep_end_us
                    self.ff_sweep_phase = "post_hold"
                    self.ff_sweep_phase_started = now
            elif self.ff_sweep_phase == "post_hold":
                self.ff_sweep_command_pct = 100.0
                self.ff_cal_current_us = self.ff_sweep_end_us
                remaining = max(self.ff_sweep_endpoint_dwell_s - (now - self.ff_sweep_phase_started), 0.0)
                if remaining <= 0.0:
                    self.add_calibration_point(
                        100.0,
                        self.ff_sweep_max_rpm,
                        self.ff_sweep_end_us,
                        f"sweep endpoint (measured {self.telemetry.rpm:.1f} RPM)",
                    )
                    self.stop_feedforward_calibration(capture=False, silent=True)
                    self.calibration_status.set(
                        "Direct-PWM sweep complete at exact 0% and 100% endpoints; "
                        "points retained locally and the throttle returned to 0%"
                    )
                    return
                self.calibration_status.set(
                    f"Board {self.ff_cal_board_id}: holding exact 100% / {self.ff_sweep_end_us:.0f} us "
                    f"({remaining:.1f} s remaining)"
                )

            self.manual_pwm_slider_us_var.set(self.ff_cal_current_us)
            self.send_pwm_bypass_us(int(round(self.ff_cal_current_us)), live=False)
            return

        settings = self._read_ff_calibration_settings()
        if settings is None:
            self.stop_feedforward_calibration(capture=False, silent=True)
            return
        dt = min(max(now - self.ff_cal_last_update, 0.0), 0.25)
        self.ff_cal_last_update = now
        error = self.ff_cal_target_rpm - self.telemetry.rpm
        deadband = settings["deadband"]
        if abs(error) > deadband:
            speed_fraction = min(max(abs(error) / max(deadband * 4.0, 1.0), 0.2), 1.0)
            direction = -1.0 if error > 0.0 else 1.0  # lower PWM opens this throttle servo
            self.ff_cal_current_us += direction * settings["rate"] * speed_fraction * dt
            self.ff_cal_current_us = min(max(self.ff_cal_current_us, 1000.0), 2000.0)
            self.ff_cal_stable_since = None
            self.ff_cal_sample_started = None
        else:
            if self.ff_cal_stable_since is None:
                self.ff_cal_stable_since = now
            if now - self.ff_cal_stable_since >= settings["stable_s"] and self.ff_cal_sample_started is None:
                self.ff_cal_sample_started = now
                self.ff_cal_recent_rpm.clear()
                self.ff_cal_recent_us.clear()

        self.manual_pwm_slider_us_var.set(self.ff_cal_current_us)
        self.send_pwm_bypass_us(int(round(self.ff_cal_current_us)), live=False)
        stable_text = "searching"
        if self.ff_cal_stable_since is not None:
            stable_text = f"inside deadband for {now - self.ff_cal_stable_since:.1f} s"
        if self.ff_cal_sample_started is not None:
            stable_text = f"sampling {now - self.ff_cal_sample_started:.1f}/{settings['sample_s']:.1f} s"
        self.calibration_status.set(f"Board {self.ff_cal_board_id}: target {self.ff_cal_target_pct:.1f}% / {self.ff_cal_target_rpm:.0f} RPM, measured {self.telemetry.rpm:.0f}, PWM {self.ff_cal_current_us:.1f} us, {stable_text}")

        if self.ff_cal_mode == "intermediate":
            if now >= self.ff_cal_point_deadline:
                failed_pct = self.ff_cal_target_pct
                self.stop_feedforward_calibration(capture=False, silent=True)
                self.calibration_status.set(f"Intermediate calibration aborted: {failed_pct:.1f}% point did not settle before the per-point timeout")
                return
            if self.ff_cal_sample_started is not None and now - self.ff_cal_sample_started >= settings["sample_s"]:
                avg_rpm = float(np.mean(self.ff_cal_recent_rpm)) if self.ff_cal_recent_rpm else self.telemetry.rpm
                avg_us = float(np.mean(self.ff_cal_recent_us)) if self.ff_cal_recent_us else self.ff_cal_current_us
                self.add_calibration_point(self.ff_cal_target_pct, avg_rpm, avg_us, "intermediate")
                self.ff_cal_current_us = avg_us
                self.ff_cal_sequence_index += 1
                self._begin_next_ff_intermediate_point()

    def stop_feedforward_calibration(self, capture: bool = False, silent: bool = False) -> None:
        mode = self.ff_cal_mode
        if capture and mode == "endpoint":
            avg_rpm = float(np.mean(self.ff_cal_recent_rpm)) if self.ff_cal_recent_rpm else self.telemetry.rpm
            avg_us = float(np.mean(self.ff_cal_recent_us)) if self.ff_cal_recent_us else self.ff_cal_current_us
            self.add_calibration_point(self.ff_cal_target_pct, self.ff_cal_target_rpm, avg_us, f"endpoint (measured {avg_rpm:.1f} RPM)")
        self.ff_cal_mode = "idle"
        self.ff_sweep_phase = "idle"
        self.ff_sweep_command_pct = 0.0
        self.ff_cal_board_id = None
        self.ff_cal_sequence = []
        self.ff_cal_sequence_index = 0
        self.ff_cal_stable_since = None
        self.ff_cal_sample_started = None
        self.manual_pwm_bypass_var.set(False)
        if self.serial_worker.is_connected():
            self.send_line("PWMBYPASS_STOP")
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.send_current_command()
        if not silent:
            action = "captured point and stopped" if capture and mode == "endpoint" else "aborted/stopped"
            self.calibration_status.set(f"Feedforward calibration {action}; throttle command returned to 0%, ARM state preserved. Model commit/reset is available live.")

    def record_calibration_sample(self, sample_time_s: float) -> None:
        if self.ff_cal_mode == "idle" or self.ff_cal_board_id != self.selected_board_id:
            return
        if self.ff_cal_mode in {"endpoint", "intermediate"}:
            if self.ff_cal_sample_started is not None or self.ff_cal_mode == "endpoint":
                self.ff_cal_recent_rpm.append(float(self.telemetry.rpm))
                self.ff_cal_recent_us.append(float(self.ff_cal_current_us))
            return
        if self.ff_cal_mode == "sweep":
            # Sweep PWM is linear in time, but the engine RPM response is not.
            # Convert measured RPM into the exact configured 0..100% RPM span;
            # plotting PWM against that normalized RPM restores the true throttle
            # curve instead of reproducing the commanded straight PWM ramp.
            if self.ff_sweep_phase != "ramp":
                return
            measured_rpm = float(self.telemetry.rpm)
            rpm_span = self.ff_sweep_max_rpm - self.ff_sweep_idle_rpm
            if rpm_span <= 1.0:
                return
            raw_pct = 100.0 * (measured_rpm - self.ff_sweep_idle_rpm) / rpm_span
            # Ignore samples outside the calibrated RPM range. Exact endpoint
            # anchors are inserted separately at 0% and 100%.
            if raw_pct <= 0.25 or raw_pct >= 99.75:
                return
            pct = min(max(raw_pct, 0.0), 100.0)
            now = time.monotonic()
            if self.ff_sweep_last_recorded_pct is not None:
                pct_change = abs(pct - self.ff_sweep_last_recorded_pct)
                elapsed = now - self.ff_sweep_last_recorded_time
                if pct_change < 0.20 and elapsed < 0.10:
                    return
            self.ff_sweep_last_recorded_pct = pct
            self.ff_sweep_last_recorded_time = now
            self.ff_calibration_points.append(
                FeedforwardCalibrationPoint(
                    pct,
                    measured_rpm,
                    float(self.ff_cal_current_us),
                    "sweep RPM-mapped",
                    True,
                )
            )
            self.ff_model_dirty = True
            self.ff_fit_cache = None
            self.calibration_dirty = True
            if len(self.ff_calibration_points) % 5 == 0:
                self.refresh_calibration_point_tree()

    def _canonical_feedforward_rpm(self, pct: float, measured_rpm: float) -> float:
        """Force 0% and 100% points to the configured target RPM endpoints."""
        try:
            if pct <= 0.05:
                return float(self.ff0_rpm_var.get())
            if pct >= 99.95:
                return float(self.ff100_rpm_var.get())
        except ValueError:
            pass
        return float(measured_rpm)

    def add_calibration_point(self, pct: float, rpm: float, us: float, source: str) -> None:
        pct = min(max(float(pct), 0.0), 100.0)
        rpm = self._canonical_feedforward_rpm(pct, rpm)
        us = min(max(float(us), 1000.0), 2000.0)
        # Replace a prior non-sweep point at the same nominal throttle rather than
        # silently accumulating conflicting endpoint/intermediate records.
        if not source.startswith("sweep"):
            self.ff_calibration_points = [p for p in self.ff_calibration_points if p.source.startswith("sweep") or abs(p.throttle_pct - pct) > 0.05]
        self.ff_calibration_points.append(FeedforwardCalibrationPoint(pct, rpm, us, source, True))
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.calibration_dirty = True
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()

    def clear_calibration_samples(self) -> None:
        if self.ff_calibration_points and not messagebox.askyesno("Erase graph points?", "Erase every local endpoint, intermediate, sweep, and pulled-model point? The engine model will not change."):
            return
        self.ff_calibration_points.clear()
        self.calibration_samples.clear()
        self.ff_fit_cache = None
        self.ff_model_dirty = True
        self.calibration_dirty = True
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.calibration_status.set("All local calibration graph points erased; engine model unchanged")
        self.update_calibration_plot()

    def refresh_calibration_point_tree(self) -> None:
        if self.ff_point_tree is None or not self.ff_point_tree.winfo_exists():
            return
        self.ff_point_tree.delete(*self.ff_point_tree.get_children())
        ordered = sorted(enumerate(self.ff_calibration_points), key=lambda item: (item[1].throttle_pct, item[0]))
        for original_index, point in ordered:
            self.ff_point_tree.insert("", "end", iid=str(original_index), values=("yes" if point.use_for_fit else "no", f"{point.throttle_pct:.3f}", f"{point.measured_rpm:.1f}", f"{point.pwm_us:.2f}", point.source))

    def _selected_point_index(self) -> Optional[int]:
        if self.ff_point_tree is None:
            return None
        selected = self.ff_point_tree.selection()
        if not selected:
            return None
        try:
            idx = int(selected[0])
        except ValueError:
            return None
        return idx if 0 <= idx < len(self.ff_calibration_points) else None

    def on_calibration_point_selected(self, _event: object = None) -> None:
        idx = self._selected_point_index()
        if idx is None:
            return
        p = self.ff_calibration_points[idx]
        self.ff_point_edit_pct_var.set(f"{p.throttle_pct:.6g}")
        self.ff_point_edit_rpm_var.set(f"{p.measured_rpm:.6g}")
        self.ff_point_edit_us_var.set(f"{p.pwm_us:.6g}")
        self.ff_point_edit_use_var.set(p.use_for_fit)

    def add_manual_calibration_point(self) -> None:
        try:
            pct = float(self.ff_point_edit_pct_var.get())
            rpm = float(self.ff_point_edit_rpm_var.get())
            us = float(self.ff_point_edit_us_var.get())
        except ValueError:
            messagebox.showerror("Bad manual point", "Throttle %, measured RPM, and PWM us must be numeric.")
            return
        if not 0.0 <= pct <= 100.0 or rpm < 0.0 or not 1000.0 <= us <= 2000.0:
            messagebox.showerror("Bad manual point", "Use throttle 0..100%, non-negative RPM, and PWM 1000..2000 us.")
            return
        self.add_calibration_point(pct, rpm, us, "manual")
        self.ff_point_edit_use_var.set(True)
        self.calibration_status.set(f"Added local manual point at {pct:.2f}% / {rpm:.1f} RPM / {us:.1f} us; engine unchanged")

    def update_selected_calibration_point(self) -> None:
        idx = self._selected_point_index()
        if idx is None:
            messagebox.showinfo("Select a point", "Select a graph point in the table first.")
            return
        try:
            pct = float(self.ff_point_edit_pct_var.get())
            rpm = float(self.ff_point_edit_rpm_var.get())
            us = float(self.ff_point_edit_us_var.get())
        except ValueError:
            messagebox.showerror("Bad point", "Throttle percentage, RPM, and PWM must be numeric.")
            return
        if not 0.0 <= pct <= 100.0 or rpm < 0.0 or not 1000.0 <= us <= 2000.0:
            messagebox.showerror("Bad point", "Point must be 0..100%, non-negative RPM, and 1000..2000 us.")
            return
        old = self.ff_calibration_points[idx]
        rpm = self._canonical_feedforward_rpm(pct, rpm)
        self.ff_calibration_points[idx] = FeedforwardCalibrationPoint(pct, rpm, us, "edited " + old.source, self.ff_point_edit_use_var.get())
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.calibration_dirty = True
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()

    def delete_selected_calibration_point(self) -> None:
        idx = self._selected_point_index()
        if idx is None:
            return
        del self.ff_calibration_points[idx]
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.calibration_dirty = True
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()

    def set_all_calibration_points_enabled(self, enabled: bool) -> None:
        if not self.ff_calibration_points:
            return
        for point in self.ff_calibration_points:
            point.use_for_fit = bool(enabled)
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.calibration_dirty = True
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()
        self.calibration_status.set(("Enabled" if enabled else "Excluded") + " all local graph points for fitting; engine unchanged")

    def toggle_selected_calibration_point(self) -> None:
        idx = self._selected_point_index()
        if idx is None:
            return
        self.ff_calibration_points[idx].use_for_fit = not self.ff_calibration_points[idx].use_for_fit
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()

    def _fit_input_points(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        selected = [p for p in self.ff_calibration_points if p.use_for_fit and (self.ff_use_sweep_var.get() or not p.source.startswith("sweep"))]
        if not selected:
            raise ValueError("No enabled calibration points are available")
        xs = np.asarray([p.throttle_pct for p in selected], dtype=float)
        ys = np.asarray([p.pwm_us for p in selected], dtype=float)
        rpms = np.asarray([self._canonical_feedforward_rpm(p.throttle_pct, p.measured_rpm) for p in selected], dtype=float)
        order = np.argsort(xs)
        xs, ys, rpms = xs[order], ys[order], rpms[order]
        # Median-bin near-identical percentage samples, especially dense sweep data.
        grouped_x: list[float] = []
        grouped_y: list[float] = []
        grouped_rpm: list[float] = []
        for key in sorted(set(round(float(x), 1) for x in xs)):
            mask = np.asarray([round(float(x), 1) == key for x in xs])
            grouped_x.append(float(np.median(xs[mask])))
            grouped_y.append(float(np.median(ys[mask])))
            grouped_rpm.append(float(np.median(rpms[mask])))
        return np.asarray(grouped_x), np.asarray(grouped_y), np.asarray(grouped_rpm)

    def fit_feedforward_model(self, show_errors: bool = True) -> Optional[dict[str, object]]:
        try:
            xs, ys, _rpms = self._fit_input_points()
            idle_rpm = float(self.ff0_rpm_var.get())
            max_rpm = float(self.ff100_rpm_var.get())
            point_count = int(float(self.ff_model_point_count_var.get()))
            order = int(float(self.ff_poly_order_var.get()))
        except (ValueError, TypeError) as exc:
            if show_errors:
                messagebox.showerror("Cannot fit model", str(exc))
            return None
        if max_rpm <= idle_rpm:
            if show_errors:
                messagebox.showerror("Cannot fit model", "100% target RPM must be greater than 0% target RPM.")
            return None
        if len(xs) < 2 or xs[0] > 0.5 or xs[-1] < 99.5:
            if show_errors:
                messagebox.showerror("Endpoint points required", "Capture or enter enabled calibration points at both 0% and 100% before fitting.")
            return None
        if not 2 <= point_count <= 12 or not 1 <= order <= 3:
            if show_errors:
                messagebox.showerror("Bad model parameters", "Point count must be 2..12 and polynomial order must be 1..3.")
            return None

        model_name = self.ff_model_type_var.get()
        grid_pct = np.linspace(0.0, 100.0, 401)
        grid_x = grid_pct / 100.0
        data_x = xs / 100.0
        y0 = float(np.median(ys[xs <= 0.5]))
        y100 = float(np.median(ys[xs >= 99.5]))
        baseline_data = y0 + (y100 - y0) * data_x
        baseline_grid = y0 + (y100 - y0) * grid_x
        fit: dict[str, object] = {"idle_rpm": idle_rpm, "max_rpm": max_rpm, "order": 1, "coefficients": [0.0] * 4, "points": []}
        try:
            if model_name == "Polynomial":
                if len(xs) < order + 1:
                    raise ValueError(f"Polynomial order {order} needs at least {order + 1} distinct points")

                # Endpoint-constrained polynomial. Every order passes exactly
                # through the measured 0% and 100% PWM values, so a least-squares
                # fit cannot extrapolate past either calibrated endpoint.
                coeffs = [y0, y100 - y0, 0.0, 0.0]
                if order >= 2:
                    z = data_x * (1.0 - data_x)
                    residual = ys - baseline_data
                    if order == 2:
                        design = z[:, np.newaxis]
                    else:
                        design = np.column_stack((z, z * data_x))
                    shape_coeffs, *_ = np.linalg.lstsq(design, residual, rcond=None)
                    a = float(shape_coeffs[0])
                    b = float(shape_coeffs[1]) if order >= 3 else 0.0
                    coeffs[1] += a
                    coeffs[2] = -a + b
                    coeffs[3] = -b

                grid_y = sum(coeffs[i] * np.power(grid_x, i) for i in range(4))
                predicted = sum(coeffs[i] * np.power(data_x, i) for i in range(4))
                fit.update({"model_type": 1, "order": order, "point_count": 0, "coefficients": coeffs, "points": []})
            elif model_name == "Piecewise linear":
                targets = np.linspace(0.0, 100.0, point_count)
                point_y = np.interp(targets, xs, ys)
                point_y[0] = y0
                point_y[-1] = y100
                model_points = [(float(x), float(y)) for x, y in zip(targets, point_y)]
                grid_y = np.interp(grid_pct, targets, point_y)
                predicted = np.interp(xs, targets, point_y)
                fit.update({"model_type": 2, "order": 1, "point_count": point_count, "points": model_points})
            else:
                grid_y = baseline_grid
                predicted = baseline_data
                fit.update({
                    "model_type": 0,
                    "order": 1,
                    "point_count": 2,
                    "points": [(0.0, y0), (100.0, y100)],
                    "coefficients": [y0, y100 - y0, 0.0, 0.0],
                })
        except (ValueError, np.linalg.LinAlgError, Warning) as exc:
            if show_errors:
                messagebox.showerror("Fit failed", str(exc))
            return None

        if np.min(grid_y) < 1000.0 or np.max(grid_y) > 2000.0:
            if show_errors:
                messagebox.showerror("Unsafe fitted model", f"Fitted PWM leaves the allowed 1000..2000 us range ({np.min(grid_y):.1f}..{np.max(grid_y):.1f} us).")
            return None
        diffs = np.diff(grid_y)
        if grid_y[-1] >= grid_y[0] - 1.0 or np.any(diffs > 1.0):
            if show_errors:
                messagebox.showerror("Non-monotonic fit", "This engine opens throttle as PWM decreases. The fitted curve must decrease monotonically from 0% to 100%.")
            return None
        rmse = float(np.sqrt(np.mean((predicted - ys) ** 2)))
        max_error = float(np.max(np.abs(predicted - ys)))
        fit["grid_pct"] = [float(v) for v in grid_pct]
        fit["grid_us"] = [float(v) for v in grid_y]
        fit["rmse_us"] = rmse
        fit["max_error_us"] = max_error
        self.ff_fit_cache = fit
        self.calibration_dirty = True
        self.calibration_status.set(f"Fit preview valid: {model_name}, RMSE {rmse:.2f} us, max point error {max_error:.2f} us, endpoints {grid_y[0]:.1f} → {grid_y[-1]:.1f} us")
        self.update_calibration_plot()
        return fit

    def send_feedforward_model(self) -> None:
        if not self.reselect_selected_board():
            return
        assert self.selected_board_id is not None
        if self.ff_model_pulled_board_id != self.selected_board_id:
            messagebox.showerror("Pull required", "Pull this selected engine's model before committing. This prevents overwriting a different engine.")
            return
        fit = self.fit_feedforward_model(show_errors=True)
        if fit is None:
            return
        model_names = {0: "Linear", 1: "Polynomial", 2: "Piecewise linear"}
        if not messagebox.askyesno(
            "Commit feedforward model?",
            f"Commit {model_names[int(fit['model_type'])]} model to selected board {self.selected_board_id}?\n\n"
            f"Target RPM: {float(fit['idle_rpm']):.0f} → {float(fit['max_rpm']):.0f}\n"
            f"Fit error: RMSE {float(fit['rmse_us']):.2f} us, max {float(fit['max_error_us']):.2f} us\n\n"
            "The controller stages all frames and applies/saves only after the final validated COMMIT. "
            "If the engine is running, the controller performs a bumpless live transfer that preserves the current PWM.",
        ):
            return
        model_type = int(fit["model_type"])
        point_count = int(fit["point_count"])
        order = int(fit["order"])
        expected_points = [(float(pct), float(int(round(float(us))))) for pct, us in fit.get("points", [])]
        self.ff_pending_commit_snapshot = {
            "board_id": self.selected_board_id,
            "model_type": model_type,
            "order": order,
            "idle_rpm": float(fit["idle_rpm"]),
            "max_rpm": float(fit["max_rpm"]),
            "coefficients": [float(v) for v in fit.get("coefficients", [])[: order + 1]],
            "points": expected_points,
        }
        commands = [
            f"SELECT {self.selected_board_id}",
            f"FFMODEL {model_type} {point_count} {order}",
            f"FFRPM {float(fit['idle_rpm']):.9g} {float(fit['max_rpm']):.9g}",
        ]
        if model_type == 1:
            commands.extend(f"FFCOEFF {i} {float(value):.9g}" for i, value in enumerate(fit["coefficients"][: order + 1]))
        else:
            commands.extend(
                f"FFPOINT {i} {float(pct):.6f} {int(round(float(us)))}"
                for i, (pct, us) in enumerate(fit["points"])
            )
        commands.append("FFCOMMIT")
        for command in commands:
            if not self.send_line(command):
                self.send_line("FFCANCEL")
                self.ff_pending_commit_snapshot = None
                self.calibration_status.set(f"Feedforward transfer stopped before commit because serial write failed at: {command}")
                messagebox.showerror("Feedforward send failed", "The staged transfer was cancelled. The active engine model was not intentionally changed.")
                return
        # Track the endpoint values expected from the committed model so any
        # main-page edits are cleared only after the same selected controller
        # reports the exact RPM endpoints and rounded PWM endpoints back.
        endpoint_expectations = (
            (self.ff0_rpm_var, float(fit["idle_rpm"])),
            (self.ff100_rpm_var, float(fit["max_rpm"])),
            (self.ff0_us_var, float(round(float(fit["grid_us"][0])))),
            (self.ff100_us_var, float(round(float(fit["grid_us"][-1])))),
        )
        for var, value in endpoint_expectations:
            key = str(var)
            self.config_dirty_vars.add(key)
            self.config_pending_values[key] = value
            self.config_pending_boards[key] = self.selected_board_id
            widget = self.config_entry_widgets.get(key)
            if widget is not None:
                try:
                    ttk.Style(self).configure("PendingConfig.TEntry", fieldbackground="#fff0b3")
                    widget.configure(style="PendingConfig.TEntry")
                except tk.TclError:
                    pass

        self.calibration_status.set(f"Staged and committed model to board {self.selected_board_id}; requesting pull-back verification")
        self.after(450, self._request_feedforward_verify)

    def _request_feedforward_verify(self) -> None:
        if self.selected_board_id is None or not self.serial_worker.is_connected():
            return
        self.ff_model_pull_requested = True
        self.ff_model_receive = {"coefficients": {}, "points": {}, "board_id": self.selected_board_id}
        self.send_line(f"SELECT {self.selected_board_id}")
        self.send_line("GETCFG")

    def reset_local_feedforward_model(self) -> None:
        try:
            idle_rpm = float(self.ff0_rpm_var.get())
            max_rpm = float(self.ff100_rpm_var.get())
            idle_us = float(self.ff0_us_var.get())
            max_us = float(self.ff100_us_var.get())
        except ValueError:
            idle_rpm, max_rpm, idle_us, max_us = 2200.0, 4250.0, 1850.0, 1450.0
        self.ff_model_type_var.set("Linear")
        self.ff_poly_order_var.set("1")
        self.ff_model_point_count_var.set("2")
        self.ff_calibration_points = [
            FeedforwardCalibrationPoint(0.0, idle_rpm, idle_us, "local reset", True),
            FeedforwardCalibrationPoint(100.0, max_rpm, max_us, "local reset", True),
        ]
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()
        self.calibration_status.set("Local values reset to a two-point linear model; engine unchanged until explicit commit")

    def reset_engine_feedforward_model(self) -> None:
        if not self.reselect_selected_board():
            return
        assert self.selected_board_id is not None
        if self.ff_model_pulled_board_id != self.selected_board_id:
            messagebox.showerror("Pull required", "Pull the selected engine before resetting its model.")
            return
        if not messagebox.askyesno(
            "Reset engine model?",
            f"Reset board {self.selected_board_id} to a linear curve using its current endpoint PWM values? "
            "This can be applied live with bumpless PWM tracking.",
        ):
            return
        self.ff_pending_commit_snapshot = {
            "board_id": self.selected_board_id,
            "model_type": 0,
            "idle_rpm": float(self.ff0_rpm_var.get()),
            "max_rpm": float(self.ff100_rpm_var.get()),
            "reset_only": True,
        }
        self.send_line(f"SELECT {self.selected_board_id}")
        self.send_line("FFRESET")
        self.calibration_status.set(f"Reset command sent to board {self.selected_board_id}; requesting pull-back verification")
        self.after(350, self._request_feedforward_verify)

    def update_calibration_plot(self) -> None:
        if self.calibration_window is None or not self.calibration_window.winfo_exists() or self.calibration_axes is None or self.calibration_canvas is None:
            return
        ax = self.calibration_axes
        rpm_ax = self.calibration_rpm_axes
        ax.clear()
        if rpm_ax is not None:
            rpm_ax.clear()
        ax.set_title("Feedforward fit: normalized target RPM / throttle → PWM")
        ax.set_xlabel("Throttle position from configured RPM endpoints (%)")
        ax.set_ylabel("Feedforward PWM (us)")
        ax.set_xlim(0.0, 100.0)
        ax.set_ylim(2000.0, 1000.0)
        ax.grid(True)
        used = [p for p in self.ff_calibration_points if p.use_for_fit]
        unused = [p for p in self.ff_calibration_points if not p.use_for_fit]
        if used:
            ax.scatter([p.throttle_pct for p in used], [p.pwm_us for p in used], s=20, label="Enabled calibration points")
            if rpm_ax is not None:
                rpm_ax.plot([p.throttle_pct for p in sorted(used, key=lambda p: p.throttle_pct)], [p.measured_rpm for p in sorted(used, key=lambda p: p.throttle_pct)], linewidth=0.8, alpha=0.45, label="Measured RPM")
        if unused:
            ax.scatter([p.throttle_pct for p in unused], [p.pwm_us for p in unused], marker="x", s=24, label="Excluded points")
        if self.ff_fit_cache is not None and "grid_pct" in self.ff_fit_cache:
            ax.plot(self.ff_fit_cache["grid_pct"], self.ff_fit_cache["grid_us"], linewidth=2.0, label="Validated fitted model")
        elif self.ff_fit_cache is not None and self.ff_fit_cache.get("points"):
            pts = self.ff_fit_cache["points"]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], linewidth=2.0, label="Pulled model")
        if not self.ff_calibration_points:
            ax.text(0.5, 0.5, "Pull a model or capture calibration points", transform=ax.transAxes, ha="center", va="center")
        if rpm_ax is not None:
            rpm_ax.set_ylabel("Measured RPM")
        if ax.lines or ax.collections:
            ax.legend(loc="upper right")
        self.calibration_figure.tight_layout()
        self.calibration_canvas.draw_idle()
        self.calibration_dirty = False

    def save_calibration_csv(self) -> None:
        if not self.ff_calibration_points:
            messagebox.showinfo("No calibration data", "No calibration points are available.")
            return
        path = filedialog.asksaveasfilename(title="Save feedforward calibration points", defaultextension=".csv", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")], initialfile="feedforward_model_points.csv")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=("throttle_pct", "measured_rpm", "pwm_us", "source", "use_for_fit"))
            writer.writeheader()
            for p in self.ff_calibration_points:
                writer.writerow({"throttle_pct": p.throttle_pct, "measured_rpm": p.measured_rpm, "pwm_us": p.pwm_us, "source": p.source, "use_for_fit": int(p.use_for_fit)})
        messagebox.showinfo("Saved", f"Saved {len(self.ff_calibration_points)} calibration points to:\n{path}")

    def load_calibration_csv(self) -> None:
        path = filedialog.askopenfilename(title="Load feedforward calibration points", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        loaded: list[FeedforwardCalibrationPoint] = []
        try:
            with open(path, "r", newline="", encoding="utf-8") as f:
                for row in csv.DictReader(f):
                    pct = float(row["throttle_pct"])
                    rpm = self._canonical_feedforward_rpm(pct, float(row["measured_rpm"]))
                    loaded.append(FeedforwardCalibrationPoint(pct, rpm, float(row["pwm_us"]), row.get("source", "CSV"), row.get("use_for_fit", "1").strip().lower() not in {"0", "false", "no"}))
        except (OSError, ValueError, KeyError) as exc:
            messagebox.showerror("Load failed", str(exc))
            return
        self.ff_calibration_points = loaded
        self.ff_model_dirty = True
        self.ff_fit_cache = None
        self.refresh_calibration_point_tree()
        self._refresh_ff_dirty_highlight()
        self.update_calibration_plot()
        self.calibration_status.set(f"Loaded {len(loaded)} local points from CSV; engine unchanged")


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
        self.hall_auto_cal_active = False
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
        detail = f"Sweeping {start:.2f}% → {end:.2f}% over {duration:.2f} s"
        self.sweep_status.set(detail)
        self.show_process_popup("throttle sweep", detail)
        self.send_current_command()

    def update_active_pattern_value(self) -> None:
        self.update_feedforward_calibration_task()
        if self.ff_cal_mode != "idle":
            return
        self.update_selected_engine_autocalibrate()
        if self.selected_autocal_active:
            return
        self.update_all_boards_endpoint_auto()
        if self.endpoint_average_active:
            self.update_endpoint_average()
        elif self.square_active:
            self.update_square_wave_value()
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
            self.close_process_popup()
        else:
            self.update_process_popup(f"Throttle sweep {alpha*100.0:.0f}% complete; current command {pct:.2f}%")

    def stop_sweep_to_zero(self) -> None:
        self.sweep_active = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.sweep_status.set("Sweep stopped; throttle forced to 0%")
        self.close_process_popup()
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
        self.hall_auto_cal_active = False
        self.cancel_endpoint_average(silent=True, zero_throttle=False)
        self.sweep_status.set("Sweep inactive")
        self.calibration_status.set("Feedforward calibration sweep inactive")
        self.square_active = True
        self.square_start_time = 0.0
        self.square_min_pct = min_pct
        self.square_max_pct = max_pct
        self.square_period_s = period_s
        self.square_snapshot_next_index = 0
        self.square_snapshot_patterns.clear()
        self.square_snapshot_dirty = True
        self.manual_throttle.set(min_pct)
        self._slider_changed()
        detail = f"Square wave active: {min_pct:.2f}% ↔ {max_pct:.2f}%, period {period_s:.3f} s; pulse centered in each captured period"
        self.square_status.set(detail)
        self.show_process_popup("square-wave PID tuning", detail)
        sent_at = self.send_current_command()
        self.square_start_time = sent_at if sent_at is not None else time.monotonic()
        self.update_snapshot_plot()

    def update_square_wave_value(self) -> None:
        if not self.square_active:
            return
        elapsed = max(time.monotonic() - self.square_start_time, 0.0)
        phase = elapsed % self.square_period_s
        high_start = 0.25 * self.square_period_s
        high_end = 0.75 * self.square_period_s
        pct = self.square_max_pct if high_start <= phase < high_end else self.square_min_pct
        self.update_process_popup(f"Square wave running; current command {pct:.2f}%")
        self.manual_throttle.set(pct)
        self._slider_changed()

    def stop_square_wave_to_zero(self) -> None:
        self.square_active = False
        self.manual_throttle.set(0.0)
        self._slider_changed()
        self.square_status.set("Square wave stopped; throttle forced to 0%")
        self.close_process_popup()
        self.send_current_command()

    @staticmethod
    def _build_step_trace(
        sample_times: deque[float],
        sample_values: deque[float],
        window_start: float,
        window_end: float,
        center: float,
    ) -> tuple[list[float], list[float]]:
        """Build a compact steps-post trace including the value at both boundaries."""
        initial_value: Optional[float] = None
        points: list[tuple[float, float]] = []
        for sample_t, sample_value in zip(sample_times, sample_values):
            if sample_t <= window_start:
                initial_value = float(sample_value)
                continue
            if sample_t > window_end:
                break
            points.append((sample_t, float(sample_value)))

        if initial_value is None and points:
            initial_value = points[0][1]
        if initial_value is None:
            return [], []

        xs = [window_start - center]
        ys = [initial_value]
        last_value = initial_value
        for sample_t, sample_value in points:
            if sample_value == last_value:
                continue
            xs.append(sample_t - center)
            ys.append(sample_value)
            last_value = sample_value
        xs.append(window_end - center)
        ys.append(last_value)
        return xs, ys

    def _find_command_edge_times(
        self,
        period_start: float,
        period_end: float,
    ) -> tuple[Optional[float], Optional[float]]:
        expected_edges = (
            (period_start + 0.25 * self.square_period_s, self.square_max_pct),
            (period_start + 0.75 * self.square_period_s, self.square_min_pct),
        )
        actual_edges: list[Optional[float]] = []
        for expected_t, expected_value in expected_edges:
            actual_t: Optional[float] = None
            for sample_t, sample_value in zip(self.command_monotonic_times, self.command_throttle_values):
                if sample_t < expected_t:
                    continue
                if sample_t > period_end:
                    break
                if abs(sample_value - expected_value) <= 1e-6:
                    actual_t = sample_t
                    break
            actual_edges.append(actual_t)
        return actual_edges[0], actual_edges[1]

    def _command_edge_delay_ms(self, period_start: float, period_end: float) -> Optional[float]:
        actual_edges = self._find_command_edge_times(period_start, period_end)
        expected_edges = (
            period_start + 0.25 * self.square_period_s,
            period_start + 0.75 * self.square_period_s,
        )
        delays_ms = [
            max(0.0, (actual_t - expected_t) * 1000.0)
            for actual_t, expected_t in zip(actual_edges, expected_edges)
            if actual_t is not None
        ]
        return max(delays_ms) if delays_ms else None

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

            high_edge_t, low_edge_t = self._find_command_edge_times(period_start, period_end)
            # Center every response on the actual transmitted high-pulse edges.
            # Fall back to the nominal center only if a command edge is missing.
            center = (
                0.5 * (high_edge_t + low_edge_t)
                if high_edge_t is not None and low_edge_t is not None else
                period_start + 0.5 * self.square_period_s
            )
            rpm_x: list[float] = []
            rpm_y: list[float] = []
            pwm_x: list[float] = []
            pwm_y: list[float] = []
            for sample_t, sample_rpm in zip(self.rpm_monotonic_times, self.rpm_monotonic_values):
                if period_start <= sample_t <= period_end:
                    rpm_x.append(sample_t - center)
                    rpm_y.append(sample_rpm)
            for sample_t, sample_pwm in zip(self.output_pwm_monotonic_times, self.output_pwm_values):
                if period_start <= sample_t <= period_end:
                    pwm_x.append(sample_t - center)
                    pwm_y.append(sample_pwm)

            command_x, command_y = self._build_step_trace(
                self.command_monotonic_times,
                self.command_throttle_values,
                period_start,
                period_end,
                center,
            )
            if len(rpm_x) >= 2:
                edge_delay_ms = self._command_edge_delay_ms(period_start, period_end)
                self.square_snapshot_patterns.append(
                    SquarePeriodSnapshot(
                        rpm_x_s=rpm_x,
                        rpm_y=rpm_y,
                        command_x_s=command_x,
                        command_pct=command_y,
                        pwm_x_s=pwm_x,
                        pwm_us=pwm_y,
                        max_command_edge_delay_ms=edge_delay_ms,
                    )
                )
                self.square_snapshot_dirty = True
                timing_text = (
                    f"; latest command edge delay ≤ {edge_delay_ms:.1f} ms"
                    if edge_delay_ms is not None else
                    "; command edge timing unavailable"
                )
                self.square_status.set(
                    f"Square wave active: captured {len(self.square_snapshot_patterns)} recent period snapshot(s){timing_text}"
                    if self.square_active else
                    f"Square wave stopped: captured {len(self.square_snapshot_patterns)} recent period snapshot(s){timing_text}"
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
        self.snapshot_pwm_axes = self.snapshot_axes.twinx()
        self.snapshot_pwm_axes.spines["right"].set_position(("outward", 68))
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
        self.snapshot_pwm_axes = None
        self.snapshot_canvas = None

    def update_snapshot_plot(self) -> None:
        if (
            self.snapshot_window is None
            or not self.snapshot_window.winfo_exists()
            or self.snapshot_axes is None
            or self.snapshot_throttle_axes is None
            or self.snapshot_pwm_axes is None
            or self.snapshot_canvas is None
        ):
            return

        self.snapshot_axes.clear()
        self.snapshot_throttle_axes.clear()
        self.snapshot_pwm_axes.clear()
        self.snapshot_pwm_axes.spines["right"].set_position(("outward", 68))
        self.snapshot_throttle_axes.yaxis.set_ticks_position("right")
        self.snapshot_throttle_axes.yaxis.set_label_position("right")
        self.snapshot_pwm_axes.yaxis.set_ticks_position("right")
        self.snapshot_pwm_axes.yaxis.set_label_position("right")
        self.snapshot_axes.set_title("RPM response aligned to actual transmitted high-pulse center")
        self.snapshot_axes.set_xlabel("Time from transmitted pulse center (s)")
        self.snapshot_axes.set_ylabel("RPM")
        self.snapshot_axes.grid(True)
        self.snapshot_throttle_axes.set_ylabel("Transmitted throttle command (%)", labelpad=10)
        self.snapshot_pwm_axes.set_ylabel("Measured throttle PWM (us)", labelpad=10)
        self.snapshot_pwm_axes.set_ylim(1000.0, 2000.0)

        period = max(self.square_period_s, 0.20)
        half = 0.5 * period
        low_lim = min(self.square_min_pct, self.square_max_pct) - 5.0
        high_lim = max(self.square_min_pct, self.square_max_pct) + 5.0
        self.snapshot_throttle_axes.set_ylim(max(-1.0, low_lim), min(101.0, high_lim))

        patterns = list(self.square_snapshot_patterns)
        if patterns:
            n = len(patterns)
            for i, snapshot in enumerate(patterns):
                alpha = 0.12 + 0.83 * ((i + 1) / n)
                width = 2.2 if i == n - 1 else 1.2
                label = "Newest RPM response" if i == n - 1 else None
                self.snapshot_axes.plot(snapshot.rpm_x_s, snapshot.rpm_y, alpha=alpha, linewidth=width, label=label)

            newest = patterns[-1]
            if newest.command_x_s:
                self.snapshot_throttle_axes.step(
                    newest.command_x_s,
                    newest.command_pct,
                    where="post",
                    linestyle="--",
                    linewidth=1.6,
                    label="Transmitted CMD throttle",
                )
            if newest.pwm_x_s:
                self.snapshot_pwm_axes.plot(
                    newest.pwm_x_s,
                    newest.pwm_us,
                    linewidth=1.5,
                    label="Measured output PWM",
                )
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

        edge_margin = min(0.15, 0.5 * period)
        self.snapshot_axes.set_xlim(-(half + edge_margin), half + edge_margin)
        if self.snapshot_throttle_axes.lines:
            self.snapshot_throttle_axes.legend(loc="upper right")
        if self.snapshot_pwm_axes.lines:
            self.snapshot_pwm_axes.legend(loc="lower right")
        self.snapshot_figure.subplots_adjust(left=0.10, right=0.76, bottom=0.14, top=0.88)
        self.snapshot_canvas.draw_idle()
        self.square_snapshot_dirty = False

    # ---------------- RPM-triggered oscilloscope ----------------
    def _read_rpm_scope_settings(self, show_errors: bool = True) -> Optional[tuple[float, float]]:
        try:
            trigger_rpm = float(self.scope_trigger_rpm_var.get())
            width_s = float(self.scope_width_s_var.get())
        except ValueError:
            if show_errors:
                messagebox.showerror("Bad RPM scope settings", "Trigger RPM and time width must be numeric.")
            return None
        if trigger_rpm <= 0.0:
            if show_errors:
                messagebox.showerror("Bad RPM trigger", "Trigger RPM must be greater than zero.")
            return None
        if not 0.20 <= width_s <= 120.0:
            if show_errors:
                messagebox.showerror("Bad scope width", "Time width must be between 0.20 and 120 seconds.")
            return None
        return trigger_rpm, width_s

    def open_and_arm_rpm_scope(self) -> None:
        self.open_rpm_scope_window()
        self.arm_rpm_scope()

    def open_rpm_scope_window(self) -> None:
        if self.scope_window is not None and self.scope_window.winfo_exists():
            self.scope_window.lift()
            self.update_rpm_scope_plot()
            return

        win = tk.Toplevel(self)
        win.title("RPM-triggered oscilloscope")
        win.geometry("1040x680")
        win.minsize(800, 520)
        win.protocol("WM_DELETE_WINDOW", self.close_rpm_scope_window)
        self.scope_window = win

        frame = ttk.Frame(win, padding=12)
        frame.pack(fill="both", expand=True)
        frame.rowconfigure(2, weight=1)
        for col in range(6):
            frame.columnconfigure(col, weight=1 if col in (1, 3) else 0)

        ttk.Label(frame, text="Trigger RPM").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.scope_trigger_rpm_var, width=12).grid(row=0, column=1, sticky="ew", padx=(6, 14))
        ttk.Label(frame, text="Time width / resolution (s)").grid(row=0, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.scope_width_s_var, width=12).grid(row=0, column=3, sticky="ew", padx=(6, 14))
        ttk.Button(frame, text="Arm trigger", command=self.arm_rpm_scope).grid(row=0, column=4, sticky="ew", padx=(0, 6))
        ttk.Button(frame, text="Stop", command=self.stop_rpm_scope).grid(row=0, column=5, sticky="ew")
        ttk.Label(frame, textvariable=self.scope_status, wraplength=960).grid(row=1, column=0, columnspan=6, sticky="ew", pady=(8, 10))

        self.scope_figure = Figure(figsize=(9.2, 5.2), dpi=100)
        self.scope_axes = self.scope_figure.add_subplot(111)
        self.scope_throttle_axes = self.scope_axes.twinx()
        self.scope_pwm_axes = self.scope_axes.twinx()
        self.scope_pwm_axes.spines["right"].set_position(("outward", 68))
        self.scope_canvas = FigureCanvasTkAgg(self.scope_figure, master=frame)
        self.scope_canvas.get_tk_widget().grid(row=2, column=0, columnspan=6, sticky="nsew")
        self.scope_dirty = True
        self.update_rpm_scope_plot()

    def close_rpm_scope_window(self) -> None:
        if self.scope_window is not None and self.scope_window.winfo_exists():
            self.scope_window.destroy()
        self.scope_window = None
        self.scope_figure = None
        self.scope_axes = None
        self.scope_throttle_axes = None
        self.scope_pwm_axes = None
        self.scope_canvas = None

    def arm_rpm_scope(self) -> None:
        parsed = self._read_rpm_scope_settings(show_errors=True)
        if parsed is None:
            return
        trigger_rpm, width_s = parsed
        self.scope_trigger_rpm = trigger_rpm
        self.scope_width_s = width_s
        self.scope_armed = True
        self.scope_triggered = False
        self.scope_trigger_time = 0.0
        self.scope_status.set(
            f"ARMED: waiting for rising RPM crossing at {trigger_rpm:.1f} RPM; total width {width_s:.3f} s ({0.5 * width_s:.3f} s pre/post)"
        )
        self.scope_dirty = True
        self.save_settings(silent=True)
        self.update_rpm_scope_plot()

    def stop_rpm_scope(self) -> None:
        self.scope_armed = False
        self.scope_triggered = False
        self.scope_trigger_time = 0.0
        self.scope_status.set("RPM scope stopped")
        self.scope_dirty = True
        self.update_rpm_scope_plot()

    def update_rpm_scope_trigger(self, now: float, previous_rpm: Optional[float], current_rpm: float) -> None:
        if self.scope_armed and not self.scope_triggered and previous_rpm is not None:
            if previous_rpm < self.scope_trigger_rpm <= current_rpm:
                self.scope_armed = False
                self.scope_triggered = True
                self.scope_trigger_time = now
                self.scope_status.set(
                    f"TRIGGERED at {current_rpm:.1f} RPM; collecting {0.5 * self.scope_width_s:.3f} s of post-trigger data"
                )
                self.scope_dirty = True

        if self.scope_triggered and now >= self.scope_trigger_time + 0.5 * self.scope_width_s:
            self.complete_rpm_scope_capture()

    def complete_rpm_scope_capture(self) -> None:
        if not self.scope_triggered or self.scope_trigger_time <= 0.0:
            return
        half = 0.5 * self.scope_width_s
        start = self.scope_trigger_time - half
        end = self.scope_trigger_time + half
        rpm_x: list[float] = []
        rpm_y: list[float] = []
        pwm_x: list[float] = []
        pwm_y: list[float] = []
        for sample_t, sample_rpm in zip(self.rpm_monotonic_times, self.rpm_monotonic_values):
            if start <= sample_t <= end:
                rpm_x.append(sample_t - self.scope_trigger_time)
                rpm_y.append(sample_rpm)
        for sample_t, sample_pwm in zip(self.output_pwm_monotonic_times, self.output_pwm_values):
            if start <= sample_t <= end:
                pwm_x.append(sample_t - self.scope_trigger_time)
                pwm_y.append(sample_pwm)
        command_x, command_y = self._build_step_trace(
            self.command_monotonic_times,
            self.command_throttle_values,
            start,
            end,
            self.scope_trigger_time,
        )
        self.scope_capture = RpmScopeCapture(
            trigger_rpm=self.scope_trigger_rpm,
            width_s=self.scope_width_s,
            rpm_x_s=rpm_x,
            rpm_y=rpm_y,
            command_x_s=command_x,
            command_pct=command_y,
            pwm_x_s=pwm_x,
            pwm_us=pwm_y,
        )
        self.scope_triggered = False
        self.scope_status.set(
            f"CAPTURED: rising crossing at {self.scope_trigger_rpm:.1f} RPM, width {self.scope_width_s:.3f} s. Press Arm trigger for another capture."
        )
        self.scope_dirty = True

    def update_rpm_scope_plot(self) -> None:
        if (
            self.scope_window is None
            or not self.scope_window.winfo_exists()
            or self.scope_axes is None
            or self.scope_throttle_axes is None
            or self.scope_pwm_axes is None
            or self.scope_canvas is None
        ):
            return

        self.scope_axes.clear()
        self.scope_throttle_axes.clear()
        self.scope_pwm_axes.clear()
        self.scope_pwm_axes.spines["right"].set_position(("outward", 68))
        self.scope_throttle_axes.yaxis.set_ticks_position("right")
        self.scope_throttle_axes.yaxis.set_label_position("right")
        self.scope_pwm_axes.yaxis.set_ticks_position("right")
        self.scope_pwm_axes.yaxis.set_label_position("right")

        width_s = self.scope_capture.width_s if self.scope_capture is not None else self.scope_width_s
        half = max(0.1, 0.5 * width_s)
        self.scope_axes.set_title("RPM-triggered single-shot oscilloscope")
        self.scope_axes.set_xlabel("Time from RPM trigger (s)")
        self.scope_axes.set_ylabel("RPM")
        self.scope_axes.set_xlim(-half, half)
        self.scope_axes.grid(True)
        self.scope_axes.axvline(0.0, linestyle=":", linewidth=1.2, label="RPM trigger")
        self.scope_throttle_axes.set_ylabel("Transmitted throttle command (%)", labelpad=10)
        self.scope_throttle_axes.set_ylim(-1.0, 101.0)
        self.scope_pwm_axes.set_ylabel("Measured throttle PWM (us)", labelpad=10)
        self.scope_pwm_axes.set_ylim(1000.0, 2000.0)

        capture = self.scope_capture
        if capture is not None:
            self.scope_axes.axhline(capture.trigger_rpm, linestyle="--", linewidth=1.0, label=f"{capture.trigger_rpm:.0f} RPM level")
            if capture.rpm_x_s:
                self.scope_axes.plot(capture.rpm_x_s, capture.rpm_y, linewidth=1.8, label="RPM")
            if capture.command_x_s:
                self.scope_throttle_axes.step(
                    capture.command_x_s,
                    capture.command_pct,
                    where="post",
                    linestyle="--",
                    linewidth=1.5,
                    label="Transmitted CMD throttle",
                )
            if capture.pwm_x_s:
                self.scope_pwm_axes.plot(capture.pwm_x_s, capture.pwm_us, linewidth=1.4, label="Measured output PWM")
        else:
            self.scope_axes.axhline(self.scope_trigger_rpm, linestyle="--", linewidth=1.0, label=f"{self.scope_trigger_rpm:.0f} RPM level")
            self.scope_axes.text(
                0.5,
                0.5,
                "Arm the trigger, then increase RPM through the selected threshold",
                transform=self.scope_axes.transAxes,
                ha="center",
                va="center",
            )

        if self.scope_axes.lines:
            self.scope_axes.legend(loc="upper left")
        if self.scope_throttle_axes.lines:
            self.scope_throttle_axes.legend(loc="upper right")
        if self.scope_pwm_axes.lines:
            self.scope_pwm_axes.legend(loc="lower right")
        self.scope_figure.subplots_adjust(left=0.10, right=0.76, bottom=0.14, top=0.88)
        self.scope_canvas.draw_idle()
        self.scope_dirty = False

    # ---------------- Telemetry / graph / logging ----------------
    def record_rpm_sample(self) -> None:
        now = time.monotonic()
        t_rel = now - self.session_t0
        previous_rpm = self.rpm_monotonic_values[-1] if self.rpm_monotonic_values else None
        self.rpm_times.append(t_rel)
        self.rpm_values.append(self.telemetry.rpm)
        self.rpm_monotonic_times.append(now)
        self.rpm_monotonic_values.append(self.telemetry.rpm)
        self.output_pwm_monotonic_times.append(now)
        self.output_pwm_values.append(float(self.telemetry.out_us))
        self.update_rpm_scope_trigger(now, previous_rpm, self.telemetry.rpm)
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
            "selected_board_id": self.selected_board_id if self.selected_board_id is not None else "",
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
        keep_s = max(keep_s, 2.0 * max(self.scope_width_s, 0.2))
        cutoff = time.monotonic() - keep_s
        while self.rpm_monotonic_times and self.rpm_monotonic_times[0] < cutoff:
            self.rpm_monotonic_times.popleft()
            self.rpm_monotonic_values.popleft()
        while self.command_monotonic_times and self.command_monotonic_times[0] < cutoff:
            self.command_monotonic_times.popleft()
            self.command_throttle_values.popleft()
        while self.output_pwm_monotonic_times and self.output_pwm_monotonic_times[0] < cutoff:
            self.output_pwm_monotonic_times.popleft()
            self.output_pwm_values.popleft()

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
        if self.scope_dirty or self.scope_triggered:
            self.update_rpm_scope_plot()
        if self.calibration_dirty or self.cal_sweep_active:
            self.update_calibration_plot()
        self.after(200, self.graph_update_loop)

    def status_update_loop(self) -> None:
        t = self.telemetry
        stale_s = time.monotonic() - t.last_rx_monotonic if t.last_rx_monotonic else 1e9
        now = time.monotonic()
        if stale_s < 1.0:
            source = "TEL" if (now - self.last_tel_a_monotonic if self.last_tel_a_monotonic else 1e9) < 1.0 else "BOARD"
            link_text = f"LIVE {source}"
        else:
            link_text = f"STALE ({stale_s:.1f}s)"
            # If selecting a board caused TEL/BOARD lines to stop, do not leave the
            # GUI stuck stale. Re-assert selection and the probe heartbeat at a slow
            # rate. This is deliberately GUI-side too, because it recovers from a
            # missed SELECT, a probe reset, or a transient CAN/serial hiccup.
            if (self.selected_board_id is not None and
                    self.serial_worker.is_connected() and
                    now - self.last_selection_repair_monotonic > 1.0):
                self.last_selection_repair_monotonic = now
                self.send_line(f"SELECT {self.selected_board_id}")
                self.send_line("PROBE")
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
            f"armed={int(t.armed_flag)} link={int(t.command_link_flag)} stationary={int(t.stationary_flag)} selected={int(bool(t.flags & 0x08))}"
        )
        self.after(250, self.status_update_loop)

    def clear_graph(self) -> None:
        self.rpm_times.clear()
        self.rpm_values.clear()
        self.rpm_monotonic_times.clear()
        self.rpm_monotonic_values.clear()
        self.command_monotonic_times.clear()
        self.command_throttle_values.clear()
        self.output_pwm_monotonic_times.clear()
        self.output_pwm_values.clear()
        self.square_snapshot_patterns.clear()
        self.scope_capture = None
        self.scope_triggered = False
        self.scope_trigger_time = 0.0
        self.scope_dirty = True
        if self.scope_armed:
            self.scope_status.set(f"ARMED: buffers cleared; waiting for {self.scope_trigger_rpm:.1f} RPM rising crossing")
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
        self.update_rpm_scope_plot()

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
                self.serial_worker.write_line("AUTO_STOP")
                self.serial_worker.write_line("HALLCAL_STOP")
                self.serial_worker.write_line("PWMBYPASS_STOP")
                self.serial_worker.write_line("FFCANCEL")
                self.serial_worker.write_line("CMD 0 0")
            except Exception:
                pass
        self.ff_cal_mode = "idle"
        self.ff_cal_board_id = None
        self.manual_pwm_bypass_var.set(False)
        self.command_armed = False
        self.sweep_active = False
        self.square_active = False
        self.cal_sweep_active = False
        self.hall_auto_cal_active = False
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
