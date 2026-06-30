#!/usr/bin/env python3
"""
PX4 motor telemetry dashboard - radio-safe monitor.

Default behavior is passive: it listens to whatever MAVLink messages are already
present on the link. It does not arm/disarm, run motor tests, override actuators,
or request continuous MAVLink streams.

The only optional transmit action is the "Start ESC polling" button. It repeatedly
sends the PX4 shell command `listener esc_status` at the selected rate and parses
the returned text. Keep the poll rate low for telemetry radios.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

import tkinter as tk
from tkinter import ttk, messagebox


APP_TITLE = "PX4 Motor Telemetry - Read Only + ESC Listener Polling"
CONFIG_FILE = os.path.expanduser("~/.px4_motor_dashboard_readonly.json")

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8


@dataclass
class DashboardConfig:
    connection: str = "udpin:0.0.0.0:14560"
    motor_count: int = 6
    stale_timeout_s: float = 15.0
    esc_can_id_base: int = 41
    esc_listener_poll_hz: float = 0.5
    temp_alarm_c: float = 90.0
    rpm_running_threshold: float = 50.0
    rpm_min: float = 0.0
    rpm_max: float = 7000.0
    rpm_green_min: float = 1800.0
    rpm_green_max: float = 6200.0
    temp_bar_max_c: float = 120.0
    voltage_bar_max_v: float = 60.0
    current_bar_max_a: float = 30.0


@dataclass
class MotorTelemetry:
    rpm: Optional[float] = None
    temperature_c: Optional[float] = None
    voltage_v: Optional[float] = None
    current_a: Optional[float] = None
    throttle_pct: Optional[float] = None
    esc_address: Optional[int] = None
    actuator_function: Optional[int] = None
    last_update: float = 0.0
    last_msg: str = "never"


def safe_float(value: Any) -> Optional[float]:
    try:
        f = float(value)
    except Exception:
        return None
    if not math.isfinite(f):
        return None
    return f


def fmt(value: Optional[float], decimals: int = 1, suffix: str = "") -> str:
    if value is None:
        return "---"
    return f"{value:.{decimals}f}{suffix}"


class MavLinkReadOnlyWorker(threading.Thread):
    def __init__(self, rx_queue: "queue.Queue[Tuple[str, Any]]", log_cb, cfg_getter):
        super().__init__(daemon=True)
        self.rx_queue = rx_queue
        self.log_cb = log_cb
        self.cfg_getter = cfg_getter
        self.stop_event = threading.Event()
        self.master = None
        self.command_queue: "queue.Queue[Tuple[List[str], float]]" = queue.Queue()
        self._seen_msg_types = set()
        self._ignored_heartbeat_sources = set()
        self._shell_poll_until = 0.0
        self._last_shell_poll = 0.0
        self._shell_line_buffer = ""
        self._listener_current_esc: Optional[int] = None
        self._listener_values: Dict[int, Dict[str, Any]] = {}

    def run(self) -> None:
        while not self.stop_event.is_set():
            cfg = self.cfg_getter()
            if mavutil is None:
                self.rx_queue.put(("error", "pymavlink is missing. Run: python3 -m pip install pymavlink"))
                time.sleep(2.0)
                continue

            if self.master is None:
                try:
                    self.log_cb(f"Opening MAVLink input: {cfg.connection}")
                    self.master = mavutil.mavlink_connection(
                        cfg.connection,
                        source_system=255,
                        source_component=190,
                        autoreconnect=True,
                        dialect="ardupilotmega",
                    )
                    self.rx_queue.put(("connection", True))
                    self.log_cb("Input open. Passive by default; ESC listener polling only starts when you press Start.")
                except Exception as exc:
                    self.rx_queue.put(("connection", False))
                    self.rx_queue.put(("error", f"Connection failed: {exc}"))
                    time.sleep(1.0)
                    continue

            self._drain_command_queue()
            self._poll_shell_if_needed()

            try:
                msg = self.master.recv_match(blocking=True, timeout=0.2)
            except Exception as exc:
                self.rx_queue.put(("error", f"MAVLink receive error: {exc}"))
                self._close_master()
                time.sleep(0.5)
                continue

            if msg is None:
                continue

            try:
                self._handle_message(msg)
            except Exception as exc:
                mtype = getattr(msg, "get_type", lambda: "?")()
                self.rx_queue.put(("error", f"Parse error for {mtype}: {exc}"))

    def close(self) -> None:
        self.stop_event.set()
        self._close_master()

    def reconnect(self) -> None:
        self._close_master()

    def _close_master(self) -> None:
        try:
            if self.master is not None:
                self.master.close()
        except Exception:
            pass
        self.master = None
        self._shell_poll_until = 0.0

    def send_shell_commands(self, commands: List[str], poll_seconds: float = 4.0) -> None:
        """Queue a small MAVLink shell command burst.

        Continuous scheduling is handled by the GUI timer so this worker stays
        simple and only sends the commands it is explicitly given.
        """
        clean = [c.strip() for c in commands if c and c.strip()]
        if clean:
            self.command_queue.put((clean, max(0.5, float(poll_seconds))))

    def _drain_command_queue(self) -> None:
        if self.master is None:
            return
        while True:
            try:
                commands, poll_seconds = self.command_queue.get_nowait()
            except queue.Empty:
                break
            for cmd in commands:
                self._send_shell_line(cmd)
                self.rx_queue.put(("log", f"PX4 shell command sent: {cmd}"))
                time.sleep(0.08)
            self._shell_poll_until = max(self._shell_poll_until, time.time() + poll_seconds)
            self._last_shell_poll = 0.0

    def _send_shell_line(self, command: str) -> None:
        if self.master is None or mavutil is None:
            self.rx_queue.put(("error", "Cannot send PX4 shell command: MAVLink connection is not open."))
            return
        if not hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_SERIAL_CONTROL"):
            self.rx_queue.put(("error", "This pymavlink build does not expose SERIAL_CONTROL."))
            return
        data = (command.rstrip() + "\n").encode("utf-8", errors="replace")
        for start in range(0, len(data), 70):
            chunk = data[start:start + 70]
            self._send_serial_control_chunk(chunk, respond=True)

    def _send_serial_control_chunk(self, chunk: bytes = b"", respond: bool = True) -> None:
        if self.master is None or mavutil is None:
            return
        flags = 0
        flags |= getattr(mavutil.mavlink, "SERIAL_CONTROL_FLAG_EXCLUSIVE", 0)
        if respond:
            flags |= getattr(mavutil.mavlink, "SERIAL_CONTROL_FLAG_RESPOND", 0)
        dev_shell = getattr(mavutil.mavlink, "SERIAL_CONTROL_DEV_SHELL", 10)
        buf = bytearray(70)
        n = min(len(chunk), 70)
        if n:
            buf[:n] = chunk[:n]
        try:
            self.master.mav.serial_control_send(dev_shell, flags, 0, 0, n, buf)
        except Exception as exc:
            self.rx_queue.put(("error", f"SERIAL_CONTROL send failed: {exc}"))

    def _poll_shell_if_needed(self) -> None:
        if self.master is None:
            return
        now = time.time()
        if now > self._shell_poll_until:
            return
        if now - self._last_shell_poll < 0.20:
            return
        self._last_shell_poll = now
        self._send_serial_control_chunk(b"", respond=True)

    def _handle_message(self, msg) -> None:
        msg_type = msg.get_type()
        now = time.time()

        if msg_type not in self._seen_msg_types:
            self._seen_msg_types.add(msg_type)
            interesting = {
                "HEARTBEAT", "ESC_STATUS", "ESC_INFO", "SYS_STATUS", "BATTERY_STATUS",
                "ACTUATOR_OUTPUT_STATUS", "ACTUATOR_CONTROL_TARGET", "SERVO_OUTPUT_RAW",
            }
            if msg_type in interesting or msg_type.startswith("ESC_TELEMETRY_"):
                self.log_cb(f"Receiving {msg_type}")

        if msg_type == "SERIAL_CONTROL":
            self._handle_serial_control(msg)
            return

        if msg_type == "HEARTBEAT":
            hb = self._parse_heartbeat(msg)
            if hb:
                self.rx_queue.put(("heartbeat", hb))
            return

        if msg_type in ("SYS_STATUS", "BATTERY_STATUS"):
            bat = self._parse_battery(msg.to_dict(), msg_type)
            if bat:
                self.rx_queue.put(("battery", bat))
            return

        if msg_type in ("ESC_STATUS", "ESC_INFO") or msg_type.startswith("ESC_TELEMETRY_"):
            for idx, values in self._parse_esc_message(msg):
                values["timestamp"] = now
                values["source"] = msg_type
                self.rx_queue.put(("motor", (idx, values)))
            return

        if msg_type in ("ACTUATOR_OUTPUT_STATUS", "ACTUATOR_CONTROL_TARGET", "SERVO_OUTPUT_RAW", "HIL_ACTUATOR_CONTROLS"):
            for idx, pct in self._parse_throttle_message(msg):
                self.rx_queue.put(("throttle", (idx, pct, now, msg_type)))

    def _handle_serial_control(self, msg) -> None:
        d = msg.to_dict()
        count = int(d.get("count", 0) or 0)
        data = d.get("data", [])
        if isinstance(data, (bytes, bytearray)):
            raw = bytes(data[:count])
        elif isinstance(data, (list, tuple)):
            raw = bytes(int(x) & 0xFF for x in data[:count])
        else:
            raw = b""
        if not raw:
            return
        text = raw.decode("utf-8", errors="replace")
        if text:
            self.rx_queue.put(("shell_text", text))
            self._parse_shell_text_for_esc_values(text)

    def _parse_shell_text_for_esc_values(self, text: str) -> None:
        self._shell_line_buffer += text
        lines = self._shell_line_buffer.splitlines(keepends=True)
        if lines and not (lines[-1].endswith("\n") or lines[-1].endswith("\r")):
            self._shell_line_buffer = lines.pop()
        else:
            self._shell_line_buffer = ""
        for raw_line in lines:
            self._parse_shell_line_for_esc_values(raw_line.strip())

    def _parse_shell_line_for_esc_values(self, line: str) -> None:
        if not line:
            return

        # PX4 listener commonly prints either:
        #   esc[0].esc_rpm: 1234
        # or a block like:
        #   esc[0]
        #     esc_rpm: 1234
        esc_inline = re.search(r"esc\s*\[\s*(\d+)\s*\]\s*(?:\.\s*([A-Za-z_][A-Za-z0-9_]*))?\s*:?\s*([-+0-9.eE]+)?", line)
        field = None
        value_text = None
        esc_idx: Optional[int] = None
        if esc_inline:
            esc_idx = int(esc_inline.group(1))
            self._listener_current_esc = esc_idx
            field = esc_inline.group(2)
            value_text = esc_inline.group(3)

        if field is None:
            esc_idx = self._listener_current_esc
            m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*([-+0-9.eE]+)", line)
            if m:
                field = m.group(1)
                value_text = m.group(2)

        if esc_idx is None or field is None or value_text is None:
            return
        value = safe_float(value_text)
        if value is None:
            return

        field = field.strip().lower()
        values = self._listener_values.setdefault(esc_idx, {})
        if field in ("esc_rpm", "rpm"):
            values["rpm"] = value
        elif field in ("esc_voltage", "voltage", "voltage_v"):
            values["voltage_v"] = self._voltage_to_v(value, "ESC_STATUS")
        elif field in ("esc_current", "current", "current_a"):
            values["current_a"] = self._current_to_a(value, "ESC_STATUS")
        elif field in ("esc_temperature", "temperature", "temperature_c"):
            values["temperature_c"] = self._temp_to_c(value, "ESC_STATUS")
        elif field in ("esc_address", "address", "node_id", "nodeid"):
            values["esc_address"] = int(value)
        elif field in ("esc_setpoint", "setpoint", "throttle", "power"):
            values["throttle_pct"] = self._power_to_pct(value)
        else:
            return

        cfg = self.cfg_getter()
        motor_index = esc_idx
        esc_address = values.get("esc_address")
        can_base = int(getattr(cfg, "esc_can_id_base", 41) or 0)
        motor_count = int(getattr(cfg, "motor_count", 6) or 6)
        if esc_address is not None and can_base <= int(esc_address) < can_base + motor_count:
            motor_index = int(esc_address) - can_base

        motor_count = 6
        if motor_index < 0 or motor_index >= motor_count:
            return

        out = dict(values)
        out["timestamp"] = time.time()
        out["source"] = "PX4 listener esc_status"
        self.rx_queue.put(("motor", (motor_index, out)))

    def _parse_heartbeat(self, msg) -> Optional[Dict[str, Any]]:
        d = msg.to_dict()
        now = time.time()
        src_system = int(getattr(msg, "get_srcSystem", lambda: 1)())
        src_component = int(getattr(msg, "get_srcComponent", lambda: 1)())
        autopilot = int(d.get("autopilot", MAV_AUTOPILOT_INVALID) or MAV_AUTOPILOT_INVALID)
        vehicle_type = int(d.get("type", 0) or 0)
        is_autopilot = vehicle_type != MAV_TYPE_GCS and autopilot != MAV_AUTOPILOT_INVALID

        if not is_autopilot:
            key = (src_system, src_component, vehicle_type, autopilot)
            if key not in self._ignored_heartbeat_sources:
                self._ignored_heartbeat_sources.add(key)
                self.log_cb(f"Ignoring non-autopilot heartbeat sys={src_system} comp={src_component}")
            return None

        base_mode = int(d.get("base_mode", 0))
        armed = False
        if mavutil is not None:
            armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return {
            "armed": armed,
            "target_system": src_system,
            "target_component": src_component,
            "last_heartbeat": now,
        }

    def _parse_battery(self, d: Dict[str, Any], msg_type: str) -> Optional[Dict[str, float]]:
        if msg_type == "SYS_STATUS":
            voltage = safe_float(d.get("voltage_battery"))
            current = safe_float(d.get("current_battery"))
            if voltage is not None and voltage >= 65535:
                voltage = None
            elif voltage is not None and abs(voltage) > 100:
                voltage /= 1000.0
            if current is not None and current <= -1:
                current = None
            elif current is not None and abs(current) > 100:
                current /= 100.0
            return {"voltage_v": voltage, "current_a": current}

        voltages = d.get("voltages")
        voltage_v = None
        if isinstance(voltages, (list, tuple)):
            valid_mv = [float(v) for v in voltages if isinstance(v, (int, float)) and 0 < v < 60000]
            if valid_mv:
                voltage_v = sum(valid_mv) / 1000.0
        current = safe_float(d.get("current_battery"))
        if current is not None:
            current /= 100.0
        return {"voltage_v": voltage_v, "current_a": current}

    def _parse_esc_message(self, msg) -> List[Tuple[int, Dict[str, Any]]]:
        d = msg.to_dict()
        msg_type = msg.get_type()

        offset = 0
        if msg_type.startswith("ESC_TELEMETRY_"):
            # Examples: ESC_TELEMETRY_1_TO_4, ESC_TELEMETRY_5_TO_8
            try:
                offset = max(0, int(msg_type.split("_")[2]) - 1)
            except Exception:
                offset = 0
        else:
            offset = int(d.get("index", 0) or 0)

        rpm = self._get_list(d, "rpm", "esc_rpm")
        voltage = self._get_list(d, "voltage", "voltages", "esc_voltage")
        current = self._get_list(d, "current", "currents", "esc_current")
        temp = self._get_list(d, "temperature", "temperatures", "esc_temperature")
        power = self._get_list(d, "power", "esc_power", "throttle", "throttle_pct")
        func = self._get_list(d, "actuator_function")
        addr = self._get_list(d, "esc_address", "address")

        count = safe_float(d.get("count", d.get("esc_count")))
        lengths = [len(x) for x in (rpm, voltage, current, temp, power, func, addr) if x]
        n = max(lengths or [int(count or 0) or 1])
        if count is not None and count > 0 and lengths:
            n = min(n, int(count)) if int(count) <= max(lengths) else n

        out = []
        cfg = self.cfg_getter()
        can_base = int(getattr(cfg, "esc_can_id_base", 41) or 0)
        motor_count = int(getattr(cfg, "motor_count", 6) or 6)

        for i in range(n):
            actuator_function = self._list_num(func, i)
            raw_index = offset + i
            esc_address = self._list_int(addr, i)

            # Mapping priority:
            # 1) PX4 actuator_function if it is present, e.g. 101 -> Motor 1.
            # 2) DroneCAN node/CAN ID, e.g. 41..46 -> Motor 1..6.
            # 3) Normal MAVLink array index, e.g. 0..5 -> Motor 1..6.
            motor_index = raw_index
            if actuator_function is not None and 101 <= int(actuator_function) <= 112:
                motor_index = int(actuator_function) - 101
            else:
                can_id_candidate = esc_address if esc_address is not None else raw_index
                if can_base > 0 and can_base <= int(can_id_candidate) < can_base + motor_count:
                    motor_index = int(can_id_candidate) - can_base
                    if esc_address is None:
                        esc_address = int(can_id_candidate)
                elif can_base > 0 and can_base <= int(can_id_candidate) < can_base + 20:
                    # Radio/PX4 may send zero-filled array slots after the real CAN IDs.
                    # Ignore IDs outside the configured motor range instead of creating
                    # fake Motor 7/8/... panels.
                    continue

            if motor_index < 0 or motor_index >= 6:
                continue

            values = {
                "rpm": self._list_num(rpm, i),
                "voltage_v": self._voltage_to_v(self._list_num(voltage, i), msg_type),
                "current_a": self._current_to_a(self._list_num(current, i), msg_type),
                "temperature_c": self._temp_to_c(self._list_num(temp, i), msg_type),
                "throttle_pct": self._power_to_pct(self._list_num(power, i)),
                "esc_address": esc_address,
                "actuator_function": int(actuator_function) if actuator_function is not None else None,
            }

            numeric = [values[k] for k in ("rpm", "voltage_v", "current_a", "temperature_c", "throttle_pct")]
            has_nonzero = any((v is not None and abs(v) > 1e-12) for v in numeric)
            count_says_present = count is not None and count > 0 and i < int(count)
            if has_nonzero or count_says_present or values["esc_address"] is not None:
                out.append((motor_index, values))
        return out

    def _parse_throttle_message(self, msg) -> List[Tuple[int, float]]:
        d = msg.to_dict()
        msg_type = msg.get_type()
        values: Iterable[Any] = []

        if msg_type == "SERVO_OUTPUT_RAW":
            raw = []
            for i in range(1, 17):
                key = f"servo{i}_raw"
                if key in d:
                    raw.append(d.get(key))
            values = raw
        else:
            for key in ("actuator", "actuators", "controls", "control"):
                if isinstance(d.get(key), (list, tuple)):
                    values = d.get(key)
                    break

        out: List[Tuple[int, float]] = []
        for idx, raw in enumerate(values):
            pct = self._norm_to_pct(raw)
            if pct is not None:
                out.append((idx, pct))
        return out

    @staticmethod
    def _get_list(d: Dict[str, Any], *keys: str) -> List[Any]:
        for key in keys:
            v = d.get(key)
            if isinstance(v, list):
                return v
            if isinstance(v, tuple):
                return list(v)
            if isinstance(v, (int, float)):
                return [v]
        return []

    @staticmethod
    def _list_num(values: List[Any], i: int) -> Optional[float]:
        if not values or i >= len(values):
            return None
        return safe_float(values[i])

    @staticmethod
    def _list_int(values: List[Any], i: int) -> Optional[int]:
        f = MavLinkReadOnlyWorker._list_num(values, i)
        return int(f) if f is not None else None

    @staticmethod
    def _voltage_to_v(v: Optional[float], msg_type: str) -> Optional[float]:
        if v is None:
            return None
        if v in (0xFFFF, 65535):
            return None
        if msg_type.startswith("ESC_TELEMETRY_"):
            # MAVLink ESC_TELEMETRY_* voltage is centivolts.
            return v / 100.0
        # PX4 ESC_STATUS normally uses volts; some sources use millivolts.
        if abs(v) > 300:
            return v / 1000.0
        return v

    @staticmethod
    def _current_to_a(v: Optional[float], msg_type: str) -> Optional[float]:
        if v is None:
            return None
        if v in (0xFFFF, 65535):
            return None
        if msg_type.startswith("ESC_TELEMETRY_"):
            # MAVLink ESC_TELEMETRY_* current is centiamps.
            return v / 100.0
        return v

    @staticmethod
    def _temp_to_c(v: Optional[float], msg_type: str) -> Optional[float]:
        if v is None:
            return None
        if v in (255, 32767, -32768):
            return None
        if msg_type.startswith("ESC_TELEMETRY_"):
            return v
        # DroneCAN raw status is Kelvin, but PX4 listener/MAVLink may already be C.
        if v > 200:
            return v - 273.15
        return v

    @staticmethod
    def _power_to_pct(v: Optional[float]) -> Optional[float]:
        if v is None:
            return None
        if -1.0 <= v <= 1.0:
            return max(0.0, min(100.0, v * 100.0))
        return max(0.0, min(100.0, v))

    @staticmethod
    def _norm_to_pct(raw: Any) -> Optional[float]:
        f = safe_float(raw)
        if f is None:
            return None
        if 900 <= f <= 2200:
            return max(0.0, min(100.0, (f - 1000.0) / 1000.0 * 100.0))
        if -1.0 <= f <= 1.0:
            return max(0.0, min(100.0, f * 100.0))
        if 0.0 <= f <= 100.0:
            return f
        return None


class RpmGauge(tk.Canvas):
    def __init__(self, parent, cfg_getter, **kwargs):
        super().__init__(parent, height=150, bg="#111827", highlightthickness=0, **kwargs)
        self.cfg_getter = cfg_getter
        self.value: Optional[float] = None
        self.overlay: Optional[Tuple[str, str]] = None
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_value(self, value: Optional[float], overlay: Optional[Tuple[str, str]] = None) -> None:
        self.value = value
        self.overlay = overlay
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        cfg = self.cfg_getter()
        w = max(20, self.winfo_width())
        h = max(20, self.winfo_height())
        cx = w / 2
        cy = h - 16
        r = min(w / 2 - 18, h - 30)
        if r < 20:
            return

        def angle(v: float) -> float:
            lo = float(getattr(cfg, "rpm_min", 0.0))
            hi = max(lo + 1.0, float(getattr(cfg, "rpm_max", 7000.0)))
            t = max(0.0, min(1.0, (v - lo) / (hi - lo)))
            return 210.0 - 240.0 * t

        # Background arc and green running band.
        self.create_arc(cx - r, cy - r, cx + r, cy + r, start=210, extent=-240,
                        style="arc", width=16, outline="#374151")
        a1 = angle(float(getattr(cfg, "rpm_green_min", 1800.0)))
        a2 = angle(float(getattr(cfg, "rpm_green_max", 6200.0)))
        self.create_arc(cx - r, cy - r, cx + r, cy + r, start=a1, extent=(a2 - a1),
                        style="arc", width=16, outline="#22c55e")

        rpm_min = float(getattr(cfg, "rpm_min", 0.0))
        rpm_max = max(rpm_min + 1.0, float(getattr(cfg, "rpm_max", 7000.0)))
        for k in range(6):
            v = rpm_min + (rpm_max - rpm_min) * k / 5
            ar = math.radians(angle(v))
            self.create_line(cx + (r - 12) * math.cos(ar), cy - (r - 12) * math.sin(ar),
                             cx + r * math.cos(ar), cy - r * math.sin(ar),
                             fill="#cbd5e1", width=2)
            label = f"{int(v/1000)}k" if v >= 1000 else str(int(v))
            self.create_text(cx + (r - 34) * math.cos(ar), cy - (r - 34) * math.sin(ar),
                             text=label, fill="#cbd5e1", font=("TkDefaultFont", 8))

        shown = rpm_min if self.value is None else max(rpm_min, min(rpm_max, float(self.value)))
        ar = math.radians(angle(shown))
        self.create_line(cx, cy, cx + (r - 30) * math.cos(ar), cy - (r - 30) * math.sin(ar),
                         fill="#f8fafc", width=4, capstyle=tk.ROUND)
        self.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#f8fafc", outline="")
        self.create_text(cx, cy - 38, text="---" if self.value is None else f"{self.value:.0f}",
                         fill="#f8fafc", font=("TkDefaultFont", 20, "bold"))
        self.create_text(cx, cy - 15, text="RPM", fill="#94a3b8", font=("TkDefaultFont", 10))

        if self.overlay:
            text, kind = self.overlay
            color = {"alert": "#dc2626", "ready": "#2563eb", "disconnected": "#7f1d1d"}.get(kind, "#334155")
            self.create_rectangle(8, 10, w - 8, 40, fill=color, outline="")
            self.create_text(w / 2, 25, text=text, fill="white", font=("TkDefaultFont", 10, "bold"))


class BarGauge(ttk.Frame):
    def __init__(self, parent, label: str, unit: str, max_getter, alarm_getter=None):
        super().__init__(parent, style="Motor.TFrame")
        self.label = label
        self.unit = unit
        self.max_getter = max_getter
        self.alarm_getter = alarm_getter
        self.value: Optional[float] = None
        self.title = ttk.Label(self, text=f"{label}: --- {unit}", style="MotorDim.TLabel")
        self.title.pack(anchor="w")
        self.canvas = tk.Canvas(self, height=18, bg="#111827", highlightthickness=0)
        self.canvas.pack(fill="x", expand=True)
        self.canvas.bind("<Configure>", lambda _e: self.redraw())

    def set_value(self, value: Optional[float]) -> None:
        self.value = value
        if value is None:
            self.title.configure(text=f"{self.label}: --- {self.unit}")
        else:
            txt = f"{value:.2f}" if abs(value) < 100 else f"{value:.0f}"
            self.title.configure(text=f"{self.label}: {txt} {self.unit}")
        self.redraw()

    def redraw(self) -> None:
        self.canvas.delete("all")
        w = max(4, self.canvas.winfo_width())
        h = max(4, self.canvas.winfo_height())
        self.canvas.create_rectangle(0, 0, w, h, fill="#1f2937", outline="#374151")
        if self.value is None:
            return
        max_val = max(0.001, float(self.max_getter() or 1.0))
        frac = max(0.0, min(1.0, abs(self.value) / max_val))
        fill = "#22c55e"
        alarm = self.alarm_getter() if self.alarm_getter else None
        if alarm is not None and self.value >= alarm:
            fill = "#ef4444"
        elif frac > 0.8:
            fill = "#f59e0b"
        self.canvas.create_rectangle(2, 2, 2 + frac * (w - 4), h - 2, fill=fill, outline="")


class ThrottleGauge(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, style="Motor.TFrame")
        self.value: Optional[float] = None
        self.title = ttk.Label(self, text="Throttle: --- %", style="MotorDim.TLabel")
        self.title.pack(anchor="w")
        self.canvas = tk.Canvas(self, height=22, bg="#111827", highlightthickness=0)
        self.canvas.pack(fill="x", expand=True)
        self.canvas.bind("<Configure>", lambda _e: self.redraw())

    def set_value(self, value: Optional[float]) -> None:
        self.value = value
        self.title.configure(text=f"Throttle: {'---' if value is None else f'{value:.0f}'} %")
        self.redraw()

    def redraw(self) -> None:
        self.canvas.delete("all")
        w = max(4, self.canvas.winfo_width())
        h = max(4, self.canvas.winfo_height())
        self.canvas.create_rectangle(0, 0, w, h, fill="#1f2937", outline="#374151")
        if self.value is None:
            return
        frac = max(0.0, min(1.0, self.value / 100.0))
        self.canvas.create_rectangle(2, 2, 2 + frac * (w - 4), h - 2, fill="#3b82f6", outline="")
        self.canvas.create_text(w / 2, h / 2, text=f"{self.value:.0f}%", fill="white", font=("TkDefaultFont", 9, "bold"))


class MotorPanelReadOnly(ttk.Frame):
    def __init__(self, parent, idx: int, cfg_getter):
        super().__init__(parent, padding=7, style="Motor.TFrame")
        self.idx = idx
        self.cfg_getter = cfg_getter

        header = ttk.Frame(self, style="Motor.TFrame")
        header.pack(fill="x")
        ttk.Label(header, text=f"Motor {idx + 1}", font=("TkDefaultFont", 13, "bold"), style="Motor.TLabel").pack(side="left")
        self.source_label = ttk.Label(header, text="never", style="MotorDim.TLabel")
        self.source_label.pack(side="right")

        self.rpm = RpmGauge(self, cfg_getter)
        self.rpm.pack(fill="both", expand=True, pady=(5, 6))

        self.temp = BarGauge(self, "Temp", "°C", lambda: self.cfg_getter().temp_bar_max_c, lambda: self.cfg_getter().temp_alarm_c)
        self.volt = BarGauge(self, "ESC V", "V", lambda: self.cfg_getter().voltage_bar_max_v)
        self.curr = BarGauge(self, "Current", "A", lambda: self.cfg_getter().current_bar_max_a)
        self.throttle = ThrottleGauge(self)
        for widget in (self.temp, self.volt, self.curr, self.throttle):
            widget.pack(fill="x", pady=2)

    def update_display(self, telemetry: MotorTelemetry, armed: bool) -> Tuple[bool, bool, bool]:
        cfg = self.cfg_getter()
        now = time.time()
        stale = (not telemetry.last_update) or ((now - telemetry.last_update) > cfg.stale_timeout_s)
        running = (telemetry.rpm or 0.0) >= cfg.rpm_running_threshold and not stale
        hot = telemetry.temperature_c is not None and telemetry.temperature_c >= cfg.temp_alarm_c and not stale

        overlay = None
        if stale:
            overlay = ("DISCONNECTED", "disconnected")
        elif hot:
            overlay = ("TEMP ALERT", "alert")
        elif armed and not running:
            overlay = ("READY", "ready")

        # Keep the last received telemetry visible even when the motor is not running
        # or when the stream goes stale. This is useful over radio links where ESC_STATUS
        # may update slowly or stop while disarmed. The overlay/source label still shows
        # stale/disconnected state, but the numeric RPM/temp/voltage/current values are
        # not blanked out.
        self.rpm.set_value(telemetry.rpm, overlay)
        self.temp.set_value(telemetry.temperature_c)
        self.volt.set_value(telemetry.voltage_v)
        self.curr.set_value(telemetry.current_a)
        self.throttle.set_value(telemetry.throttle_pct)

        age = "never" if not telemetry.last_update else ("stale" if stale else f"{now - telemetry.last_update:.1f}s")
        extra = ""
        if telemetry.esc_address is not None:
            extra += f" addr={telemetry.esc_address}"
        if telemetry.actuator_function is not None:
            extra += f" func={telemetry.actuator_function}"
        self.source_label.configure(text=f"{telemetry.last_msg}{extra} · {age}")
        return stale, running, hot


class DashboardApp:
    def __init__(self, root: tk.Tk, cli_connection: Optional[str] = None):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1360x840")
        self.root.minsize(1050, 620)

        self.cfg = self._load_config()
        if cli_connection:
            self.cfg.connection = cli_connection

        self.cfg.motor_count = 6
        self.motors = [MotorTelemetry() for _ in range(6)]
        self.rx_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self.esc_polling = False
        self._esc_poll_after_id: Optional[str] = None
        self.connected = False
        self.armed = False
        self.last_heartbeat = 0.0
        self.global_voltage: Optional[float] = None
        self.global_current: Optional[float] = None

        self._setup_style()
        self._build_ui()

        self.worker = MavLinkReadOnlyWorker(self.rx_queue, self.log_threadsafe, lambda: self.cfg)
        self.worker.start()

        self._poll_queue()
        self._ui_tick()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _load_config(self) -> DashboardConfig:
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            cfg = DashboardConfig()
            for key, value in data.items():
                if hasattr(cfg, key):
                    setattr(cfg, key, value)
            return cfg
        except Exception:
            return DashboardConfig()

    def _save_config(self) -> None:
        self.cfg.connection = self.connection_var.get().strip() or self.cfg.connection
        try:
            self.cfg.motor_count = 6
            self.cfg.stale_timeout_s = max(0.5, float(self.stale_timeout_var.get()))
            self.cfg.temp_alarm_c = float(self.temp_alarm_var.get())
            self.cfg.rpm_running_threshold = float(self.rpm_threshold_var.get())
            self.cfg.esc_can_id_base = int(self.can_id_base_var.get())
            self.cfg.esc_listener_poll_hz = max(0.05, min(2.0, float(self.esc_poll_hz_var.get())))
        except Exception:
            pass
        try:
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(self.cfg.__dict__, f, indent=2)
        except Exception as exc:
            self.log(f"Config save failed: {exc}")

    def _setup_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        self.root.configure(bg="#0f172a")
        style.configure("TFrame", background="#0f172a")
        style.configure("Motor.TFrame", background="#111827")
        style.configure("TLabel", background="#0f172a", foreground="#e5e7eb")
        style.configure("Motor.TLabel", background="#111827", foreground="#e5e7eb")
        style.configure("MotorDim.TLabel", background="#111827", foreground="#94a3b8")
        style.configure("TLabelframe", background="#0f172a", foreground="#e5e7eb")
        style.configure("TLabelframe.Label", background="#0f172a", foreground="#e5e7eb")
        style.configure("TButton", padding=6)
        style.configure("Treeview", rowheight=28)

    def _build_ui(self) -> None:
        self.connection_var = tk.StringVar(value=self.cfg.connection)
        self.motor_count_var = tk.IntVar(value=6)
        self.stale_timeout_var = tk.DoubleVar(value=self.cfg.stale_timeout_s)
        self.temp_alarm_var = tk.DoubleVar(value=self.cfg.temp_alarm_c)
        self.rpm_threshold_var = tk.DoubleVar(value=self.cfg.rpm_running_threshold)
        self.can_id_base_var = tk.IntVar(value=self.cfg.esc_can_id_base)
        self.esc_poll_hz_var = tk.DoubleVar(value=getattr(self.cfg, "esc_listener_poll_hz", 0.5))

        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="MAVLink input:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.connection_var).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(top, text="Reconnect", command=self.reconnect).grid(row=0, column=2, padx=3)
        ttk.Button(top, text="Save", command=self.save_clicked).grid(row=0, column=3, padx=3)
        ttk.Button(top, text="Clear Log", command=lambda: self.log_text.delete("1.0", "end")).grid(row=0, column=4, padx=3)
        self.esc_poll_button = ttk.Button(top, text="Start ESC polling", command=self.toggle_esc_listener_polling)
        self.esc_poll_button.grid(row=0, column=5, padx=3)

        opts = ttk.LabelFrame(self.root, text="Read-only options", padding=8)
        opts.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(opts, text="Motors: 1–6 only").grid(row=0, column=0, sticky="w", padx=(0, 14))
        self._entry_row(opts, "Poll Hz", self.esc_poll_hz_var, 0, 1)
        self._entry_row(opts, "Stale s", self.stale_timeout_var, 0, 3)
        self._entry_row(opts, "Temp alarm °C", self.temp_alarm_var, 0, 5)
        self._entry_row(opts, "Running RPM", self.rpm_threshold_var, 0, 7)
        self._entry_row(opts, "CAN base", self.can_id_base_var, 0, 9)
        ttk.Label(opts, text="No arm/disarm, no motor commands. ESC polling only sends: listener esc_status").grid(row=0, column=11, sticky="w", padx=(18, 0))

        status = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        status.pack(fill="x")
        self.status_frame = tk.Frame(status, bg="#991b1b", padx=10, pady=8)
        self.status_frame.pack(fill="x")
        self.status_label = tk.Label(self.status_frame, text="NO HEARTBEAT", bg="#991b1b", fg="white", font=("TkDefaultFont", 20, "bold"))
        self.status_label.pack(side="left")
        self.detail_label = tk.Label(self.status_frame, text=" waiting for autopilot heartbeat", bg="#991b1b", fg="white")
        self.detail_label.pack(side="left", padx=(12, 0))

        split = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        split.pack(fill="both", expand=True)
        split.columnconfigure(0, weight=5)
        split.columnconfigure(1, weight=2)
        split.rowconfigure(0, weight=1)

        self.motor_grid = ttk.Frame(split)
        self.motor_grid.grid(row=0, column=0, sticky="nsew")
        for c in range(3):
            self.motor_grid.columnconfigure(c, weight=1)
        for r in range(4):
            self.motor_grid.rowconfigure(r, weight=1)

        self.motor_panels: List[MotorPanelReadOnly] = []
        self._rebuild_motor_panels()

        side = ttk.Frame(split)
        side.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        side.rowconfigure(2, weight=1)
        ttk.Label(side, text="Global battery").grid(row=0, column=0, sticky="w")
        self.battery_label = ttk.Label(side, text="--- V   --- A")
        self.battery_label.grid(row=1, column=0, sticky="w", pady=(2, 8))
        self.log_text = tk.Text(side, height=18, bg="#020617", fg="#e5e7eb", insertbackground="#e5e7eb", wrap="word", relief="flat")
        self.log_text.grid(row=2, column=0, sticky="nsew")
        self.log("Dashboard started in radio-safe mode with RPM needle gauges, persistent values, CAN-ID mapping, and ESC listener polling.")
        self.log("Default behavior is passive. Press Start ESC polling to repeatedly send: listener esc_status.")
        self.log("Default CAN ID mapping is 41..46 -> Motor 1..6. Motors 7 and 8 are ignored.")

    def _entry_row(self, parent, label, var, row, col):
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(0, 4))
        ttk.Entry(parent, textvariable=var, width=8).grid(row=row, column=col + 1, sticky="w", padx=(0, 10))

    def reconnect(self):
        self._save_config()
        self.connected = False
        self.last_heartbeat = 0.0
        try:
            self.worker.reconnect()
        except Exception:
            pass
        self.log(f"Reconnect requested: {self.cfg.connection}")

    def toggle_esc_listener_polling(self):
        self._save_config_light()
        if self.esc_polling:
            self.esc_polling = False
            try:
                if self._esc_poll_after_id is not None:
                    self.root.after_cancel(self._esc_poll_after_id)
            except Exception:
                pass
            self._esc_poll_after_id = None
            self.esc_poll_button.configure(text="Start ESC polling")
            self.log("Stopped PX4 ESC listener polling")
            return

        self.esc_polling = True
        self.esc_poll_button.configure(text="Stop ESC polling")
        self.log(f"Started PX4 ESC listener polling at {self.cfg.esc_listener_poll_hz:.2f} Hz")
        self._esc_poll_tick()

    def _esc_poll_tick(self):
        if not self.esc_polling:
            return
        self._save_config_light()
        hz = max(0.05, min(2.0, float(getattr(self.cfg, "esc_listener_poll_hz", 0.5))))
        interval_ms = int(max(500, round(1000.0 / hz)))
        poll_seconds = max(1.0, min(3.0, interval_ms / 1000.0 * 0.8))
        try:
            self.worker.send_shell_commands(["listener esc_status"], poll_seconds=poll_seconds)
        except Exception as exc:
            self.log(f"Could not request PX4 ESC listener: {exc}")
        self._esc_poll_after_id = self.root.after(interval_ms, self._esc_poll_tick)

    def save_clicked(self):
        self._save_config()
        self.cfg.motor_count = 6
        if len(self.motors) != 6:
            self._resize_motors(6)
        self.log("Config saved")

    def _resize_motors(self, count: int):
        count = 6
        if count > len(self.motors):
            for _ in range(count - len(self.motors)):
                self.motors.append(MotorTelemetry())
        else:
            self.motors = self.motors[:count]
        self.cfg.motor_count = count
        self._rebuild_motor_panels()

    def _rebuild_motor_panels(self):
        if not hasattr(self, "motor_grid"):
            return
        for panel in getattr(self, "motor_panels", []):
            try:
                panel.destroy()
            except Exception:
                pass
        self.motor_panels = []
        count = len(self.motors)
        cols = 3 if count > 2 else max(1, count)
        for c in range(4):
            self.motor_grid.columnconfigure(c, weight=0)
        for c in range(cols):
            self.motor_grid.columnconfigure(c, weight=1)
        rows = max(1, math.ceil(count / cols))
        for r in range(rows):
            self.motor_grid.rowconfigure(r, weight=1)
        for i in range(count):
            p = MotorPanelReadOnly(self.motor_grid, i, lambda: self.cfg)
            p.grid(row=i // cols, column=i % cols, sticky="nsew", padx=5, pady=5)
            self.motor_panels.append(p)

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "motor":
                    idx, values = payload
                    self._handle_motor_update(idx, values)
                elif kind == "throttle":
                    idx, pct, ts, source = payload
                    self._handle_throttle_update(idx, pct, ts, source)
                elif kind == "heartbeat":
                    self._handle_heartbeat(payload)
                elif kind == "battery":
                    self._handle_battery(payload)
                elif kind == "connection":
                    self.connected = bool(payload)
                elif kind == "error":
                    self.log(f"ERROR: {payload}")
                elif kind == "shell_text":
                    self.log_shell_text(str(payload))
                elif kind == "log":
                    self.log(str(payload))
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _handle_motor_update(self, idx: int, values: Dict[str, Any]):
        if idx < 0 or idx >= 6 or idx >= len(self.motors):
            return
        m = self.motors[idx]
        for attr, key in [
            ("rpm", "rpm"),
            ("temperature_c", "temperature_c"),
            ("voltage_v", "voltage_v"),
            ("current_a", "current_a"),
            ("throttle_pct", "throttle_pct"),
        ]:
            v = values.get(key)
            if v is not None:
                setattr(m, attr, float(v))
        if values.get("esc_address") is not None:
            m.esc_address = int(values["esc_address"])
        if values.get("actuator_function") is not None:
            m.actuator_function = int(values["actuator_function"])
        m.last_update = float(values.get("timestamp", time.time()))
        m.last_msg = str(values.get("source", "MAVLink"))

    def _handle_throttle_update(self, idx: int, pct: float, timestamp: float, source: str):
        if idx < 0 or idx >= len(self.motors):
            return
        m = self.motors[idx]
        m.throttle_pct = float(pct)
        if not m.last_update:
            m.last_update = timestamp
            m.last_msg = source

    def _handle_heartbeat(self, payload: Dict[str, Any]):
        prev = self.armed
        self.armed = bool(payload.get("armed", False))
        self.connected = True
        self.last_heartbeat = float(payload.get("last_heartbeat", time.time()))
        if prev != self.armed:
            self.log("PX4 armed" if self.armed else "PX4 disarmed")

    def _handle_battery(self, payload: Dict[str, Any]):
        self.global_voltage = payload.get("voltage_v")
        self.global_current = payload.get("current_a")

    def _ui_tick(self):
        self._save_config_light()
        now = time.time()
        heartbeat_age = now - self.last_heartbeat if self.last_heartbeat else 9999.0
        heartbeat_ok = heartbeat_age < 5.0
        if not heartbeat_ok:
            self.connected = False

        stale_count = 0
        running_count = 0
        hot_count = 0
        if len(getattr(self, "motor_panels", [])) != len(self.motors):
            self._rebuild_motor_panels()
        for i, m in enumerate(self.motors):
            if i >= len(self.motor_panels):
                break
            stale, running, hot = self.motor_panels[i].update_display(m, self.armed)
            stale_count += int(stale)
            running_count += int(running)
            hot_count += int(hot)

        if not heartbeat_ok:
            color, status, detail = "#991b1b", "NO HEARTBEAT", f"{heartbeat_age:.1f}s since autopilot heartbeat"
        elif self.armed:
            color = "#16a34a" if running_count else "#2563eb"
            status = "ARMED / RUNNING" if running_count else "ARMED / IDLE"
            detail = f"{running_count}/{len(self.motors)} running · {stale_count} stale · read-only"
        else:
            color, status, detail = "#dc2626", "DISARMED", f"connected · {stale_count} stale · read-only"
        if hot_count:
            color, status = "#dc2626", f"TEMP ALERT ({hot_count})"

        self.status_frame.configure(bg=color)
        self.status_label.configure(text=status, bg=color)
        self.detail_label.configure(text=detail, bg=color)
        self.battery_label.configure(text=f"{fmt(self.global_voltage, 2, ' V')}   {fmt(self.global_current, 2, ' A')}")
        self.root.after(250, self._ui_tick)

    def _save_config_light(self):
        # Keep runtime values current without writing the file every tick.
        self.cfg.connection = self.connection_var.get().strip() or self.cfg.connection
        try:
            self.cfg.motor_count = 6
            self.cfg.stale_timeout_s = max(0.5, float(self.stale_timeout_var.get()))
            self.cfg.temp_alarm_c = float(self.temp_alarm_var.get())
            self.cfg.rpm_running_threshold = float(self.rpm_threshold_var.get())
            self.cfg.esc_can_id_base = int(self.can_id_base_var.get())
            self.cfg.esc_listener_poll_hz = max(0.05, min(2.0, float(self.esc_poll_hz_var.get())))
        except Exception:
            pass

    def log_shell_text(self, text: str):
        for line in text.replace("\r", "\n").split("\n"):
            line = line.rstrip()
            if line:
                self.log(f"PX4> {line}")

    def log_threadsafe(self, msg: str):
        self.rx_queue.put(("log", msg))

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        try:
            self.log_text.insert("end", f"[{ts}] {msg}\n")
            self.log_text.see("end")
            lines = int(self.log_text.index("end-1c").split(".")[0])
            if lines > 500:
                self.log_text.delete("1.0", "80.0")
        except Exception:
            pass

    def _on_close(self):
        self.esc_polling = False
        try:
            if self._esc_poll_after_id is not None:
                self.root.after_cancel(self._esc_poll_after_id)
        except Exception:
            pass
        try:
            self._save_config()
        except Exception:
            pass
        try:
            self.worker.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="PX4 read-only motor telemetry dashboard")
    parser.add_argument("--connection", "--connect", dest="connection", default=None,
                        help="pymavlink connection string, e.g. udpin:0.0.0.0:14560 or /dev/ttyUSB0,57600")
    args = parser.parse_args()

    root = tk.Tk()
    DashboardApp(root, cli_connection=args.connection)
    root.mainloop()


if __name__ == "__main__":
    main()
