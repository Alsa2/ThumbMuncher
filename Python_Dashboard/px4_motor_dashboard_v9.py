#!/usr/bin/env python3
"""
PX4/MAVProxy six-motor dashboard.

Designed for:
  mavproxy.py --master=/dev/ttyACM0,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560

Recommended dashboard endpoint:
  udpin:0.0.0.0:14560

Key idea:
- Use normal MAVLink messages when PX4 streams them.
- Also use PX4 MAVLink Shell commands for the things that you already verified work in QGC:
    commander arm -f
    listener esc_status
    listener actuator_motors
    actuator_test set ...
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import re
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

import tkinter as tk
from tkinter import ttk, filedialog, messagebox


APP_TITLE = "PX4 Six Motor Dashboard"
CONFIG_FILE = os.path.expanduser("~/.px4_motor_dashboard.json")

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_DO_SET_ACTUATOR = 187
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_ESC_STATUS = 291
MAVLINK_MSG_ID_ESC_INFO = 290

SERIAL_CONTROL_DEV_SHELL_FALLBACK = 10
SERIAL_CONTROL_FLAG_REPLY_FALLBACK = 1
SERIAL_CONTROL_FLAG_RESPOND_FALLBACK = 2
SERIAL_CONTROL_FLAG_EXCLUSIVE_FALLBACK = 4

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8


@dataclass
class DashboardConfig:
    connection: str = "udpin:0.0.0.0:14560"
    motor_count: int = 6

    # Polling / stale behavior
    esc_shell_poll_hz: float = 2.0
    throttle_shell_poll_hz: float = 2.0
    heartbeat_request_hz: float = 1.0
    esc_mavlink_request_hz: float = 10.0
    stale_timeout_s: float = 2.5
    heartbeat_timeout_s: float = 4.0

    # Sources
    poll_esc_status_shell: bool = True
    poll_actuator_motors_shell: bool = True
    parse_mavlink_esc_status: bool = True

    # Gauges
    rpm_min: float = 0.0
    rpm_max: float = 5000.0
    rpm_green_min: float = 1800.0
    rpm_green_max: float = 4250.0
    rpm_running_threshold: float = 50.0

    temp_alarm_c: float = 90.0
    temp_bar_max_c: float = 120.0
    voltage_bar_max_v: float = 60.0
    current_bar_max_a: float = 30.0

    # IMPORTANT: default false because your MAVLink battery/status voltage was misleading.
    fill_missing_esc_voltage_from_battery: bool = False

    # Control
    use_shell_for_arm: bool = True
    prefer_shell_actuator_test: bool = True
    override_send_hz: float = 4.0
    actuator_test_duration_s: float = 1.0
    actuator_motors_topic: str = "actuator_motors"

    # Logging
    log_to_csv: bool = False
    csv_path: str = "px4_motor_dashboard_log.csv"


@dataclass
class MotorTelemetry:
    rpm: Optional[float] = None
    temperature_c: Optional[float] = None
    voltage_v: Optional[float] = None
    current_a: Optional[float] = None
    throttle_pct: Optional[float] = None
    last_update: float = 0.0
    last_msg: str = "never"
    esc_address: Optional[int] = None
    actuator_function: Optional[int] = None
    temp_alert_active: bool = False
    stale_announced: bool = False


def safe_float(value: Any) -> Optional[float]:
    try:
        f = float(value)
    except Exception:
        return None
    if not math.isfinite(f):
        return None
    return f


class MavLinkWorker(threading.Thread):
    def __init__(self, rx_queue: "queue.Queue[Tuple[str, Any]]", log_cb, config_getter):
        super().__init__(daemon=True)
        self.rx_queue = rx_queue
        self.log_cb = log_cb
        self.config_getter = config_getter

        self.stop_event = threading.Event()
        self.master = None
        self._lock = threading.Lock()

        self.target_system = 1
        self.target_component = 1
        self.last_heartbeat = 0.0
        self._seen_msg_types = set()
        self._ignored_heartbeat_sources = set()

        self._last_esc_shell_poll = 0.0
        self._last_throttle_shell_poll = 0.0
        self._last_stream_request = 0.0
        self._shell_buffer = ""
        self._last_esc_key = None
        self._last_throttle_key = None

    def run(self) -> None:
        while not self.stop_event.is_set():
            cfg = self.config_getter()
            if mavutil is None:
                self.rx_queue.put(("error", "pymavlink is missing. Run: python3 -m pip install pymavlink"))
                time.sleep(2.0)
                continue

            if self.master is None:
                try:
                    self.log_cb(f"Connecting to MAVLink: {cfg.connection}")
                    m = mavutil.mavlink_connection(
                        cfg.connection,
                        source_system=255,
                        source_component=190,
                        autoreconnect=True,
                        dialect="ardupilotmega",
                    )
                    with self._lock:
                        self.master = m
                    self.rx_queue.put(("connection", True))
                    self.log_cb("MAVLink socket open; waiting for PX4 heartbeat")
                except Exception as exc:
                    self.rx_queue.put(("connection", False))
                    self.rx_queue.put(("error", f"Connection failed: {exc}"))
                    time.sleep(1.0)
                    continue

            try:
                msg = self.master.recv_match(blocking=True, timeout=0.08)
            except Exception as exc:
                self.rx_queue.put(("error", f"MAVLink receive error: {exc}"))
                with self._lock:
                    try:
                        if self.master:
                            self.master.close()
                    except Exception:
                        pass
                    self.master = None
                time.sleep(0.5)
                continue

            if msg is not None:
                try:
                    self._handle_message(msg)
                except Exception as exc:
                    mtype = getattr(msg, "get_type", lambda: "?")()
                    self.rx_queue.put(("error", f"Parse error for {mtype}: {exc}"))

            try:
                self._poll_shell_topics(cfg)
                self._maybe_request_mavlink_streams(cfg)
            except Exception as exc:
                self.rx_queue.put(("error", f"Polling/request error: {exc}"))
                time.sleep(0.2)

    def close(self) -> None:
        self.stop_event.set()
        with self._lock:
            if self.master is not None:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None

    def _handle_message(self, msg) -> None:
        msg_type = msg.get_type()
        now = time.time()

        if msg_type not in self._seen_msg_types:
            self._seen_msg_types.add(msg_type)
            if msg_type in {"HEARTBEAT", "ESC_STATUS", "ESC_INFO", "SYS_STATUS", "BATTERY_STATUS", "SERIAL_CONTROL"}:
                self.log_cb(f"Receiving {msg_type}")

        if msg_type == "HEARTBEAT":
            d = msg.to_dict()
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
                return

            base_mode = int(d.get("base_mode", 0))
            armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.target_system = src_system
            self.target_component = src_component if src_component > 0 else 1
            self.last_heartbeat = now
            self.rx_queue.put(("heartbeat", {
                "armed": armed,
                "target_system": self.target_system,
                "target_component": self.target_component,
                "last_heartbeat": now,
            }))
            return

        if msg_type in ("SYS_STATUS", "BATTERY_STATUS"):
            bat = self._parse_battery(msg.to_dict(), msg_type)
            if bat:
                self.rx_queue.put(("battery", bat))

        if msg_type == "SERIAL_CONTROL":
            self._handle_serial_control(msg)

        if msg_type in ("ESC_STATUS", "ESC_INFO") or msg_type.startswith("ESC_TELEMETRY_"):
            if self.config_getter().parse_mavlink_esc_status:
                for idx, values in self._parse_esc_mavlink(msg):
                    values["timestamp"] = now
                    values["source"] = msg_type
                    self.rx_queue.put(("motor", (idx, values)))

        if msg_type == "COMMAND_ACK":
            self.rx_queue.put(("ack", msg.to_dict()))

        if msg_type == "PARAM_EXT_ACK":
            self.rx_queue.put(("log", f"PARAM_EXT_ACK: {msg.to_dict()}"))

    def _poll_shell_topics(self, cfg: DashboardConfig) -> None:
        now = time.time()
        if cfg.poll_esc_status_shell and cfg.esc_shell_poll_hz > 0:
            period = 1.0 / max(0.1, cfg.esc_shell_poll_hz)
            if now - self._last_esc_shell_poll >= period:
                self._last_esc_shell_poll = now
                self.send_px4_shell_command("listener esc_status")

        if cfg.poll_actuator_motors_shell and cfg.throttle_shell_poll_hz > 0:
            period = 1.0 / max(0.1, cfg.throttle_shell_poll_hz)
            if now - self._last_throttle_shell_poll >= period:
                self._last_throttle_shell_poll = now
                topic = (cfg.actuator_motors_topic or "actuator_motors").strip()
                self.send_px4_shell_command(f"listener {topic}")

    def _maybe_request_mavlink_streams(self, cfg: DashboardConfig) -> None:
        now = time.time()
        period = 10.0
        if now - self._last_stream_request < period:
            return
        self._last_stream_request = now

        if cfg.heartbeat_request_hz > 0:
            self._request_message_interval(MAVLINK_MSG_ID_HEARTBEAT, cfg.heartbeat_request_hz, quiet=True)
        if cfg.esc_mavlink_request_hz > 0:
            self._request_message_interval(MAVLINK_MSG_ID_ESC_STATUS, cfg.esc_mavlink_request_hz, quiet=True)
            self._request_message_interval(MAVLINK_MSG_ID_ESC_INFO, max(1.0, min(5.0, cfg.esc_mavlink_request_hz)), quiet=True)

    def request_message_interval_now(self, msg_id: int, rate_hz: float) -> None:
        self._request_message_interval(msg_id, rate_hz, quiet=False)

    def _request_message_interval(self, msg_id: int, rate_hz: float, quiet: bool = False) -> None:
        if rate_hz <= 0:
            interval_us = -1
        else:
            interval_us = int(1_000_000 / max(0.1, rate_hz))
        try:
            self._send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, [msg_id, interval_us, 0, 0, 0, 0, 0])
            if not quiet:
                self.log_cb(f"Requested MAVLink message {msg_id} at {rate_hz:g} Hz")
        except Exception as exc:
            if not quiet:
                self.log_cb(f"Message interval request failed: {exc}")

    def _parse_battery(self, d: Dict[str, Any], msg_type: str) -> Optional[Dict[str, float]]:
        # Stored only as global battery; it is NOT used for ESC voltage unless the config says so.
        if msg_type == "SYS_STATUS":
            voltage = safe_float(d.get("voltage_battery"))
            current = safe_float(d.get("current_battery"))

            # PX4 uses 65535 mV to mean "battery voltage unknown". Do not convert
            # that into a fake 65.535 V value.
            if voltage is not None and voltage >= 65535:
                voltage = None
            elif voltage is not None and abs(voltage) > 100:
                voltage /= 1000.0

            # PX4 uses -1 to mean unknown current.
            if current is not None and current <= -1:
                current = None
            elif current is not None and abs(current) > 100:
                current /= 100.0

            return {"voltage_v": voltage, "current_a": current, "source": msg_type}

        voltages = d.get("voltages")
        voltage_v = None
        if isinstance(voltages, (list, tuple)):
            valid_mv = [float(v) for v in voltages if isinstance(v, (int, float)) and 0 < v < 60000]
            if valid_mv:
                voltage_v = sum(valid_mv) / 1000.0
        current = safe_float(d.get("current_battery"))
        if current is not None:
            current /= 100.0
        return {"voltage_v": voltage_v, "current_a": current, "source": msg_type}

    def _parse_esc_mavlink(self, msg) -> List[Tuple[int, Dict[str, Any]]]:
        d = msg.to_dict()
        msg_type = msg.get_type()
        offset = 0
        if msg_type.startswith("ESC_TELEMETRY_"):
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
        for i in range(n):
            actuator_function = self._list_num(func, i)
            motor_index = offset + i
            if actuator_function is not None and 101 <= int(actuator_function) <= 112:
                motor_index = int(actuator_function) - 101

            values = {
                "rpm": self._list_num(rpm, i),
                "voltage_v": self._voltage_to_v(self._list_num(voltage, i)),
                "current_a": self._current_to_a(self._list_num(current, i)),
                "temperature_c": self._temp_to_c(self._list_num(temp, i)),
                "throttle_pct": self._power_to_pct(self._list_num(power, i)),
                "esc_address": self._list_int(addr, i),
                "actuator_function": int(actuator_function) if actuator_function is not None else None,
            }

            # Do not mark empty zero-filled MAVLink array slots as connected. PX4's
            # ESC_STATUS message is array-based, so unused slots often arrive as
            # rpm=0, voltage=0, current=0. A connected idle ESC will normally be
            # confirmed by shell listener timestamp/temp/current or by a nonzero field.
            numeric = [values[k] for k in ("rpm", "voltage_v", "current_a", "temperature_c", "throttle_pct")]
            has_nonzero = any((v is not None and abs(v) > 1e-12) for v in numeric)
            count_says_present = count is not None and count > 0 and i < int(count)
            if has_nonzero or count_says_present:
                out.append((motor_index, values))
        return out

    def _handle_serial_control(self, msg) -> None:
        d = msg.to_dict()
        count = int(d.get("count", 0) or 0)
        data = d.get("data", [])
        if count <= 0 or not data:
            return

        try:
            payload = bytes(int(x) & 0xFF for x in list(data)[:count]).decode("utf-8", errors="ignore")
        except Exception:
            return

        if not payload:
            return

        self._shell_buffer = (self._shell_buffer + payload)[-30000:]

        esc = self._parse_shell_esc_status(self._shell_buffer)
        if esc:
            key, updates = esc
            if key != self._last_esc_key:
                self._last_esc_key = key
                now = time.time()
                for motor_index, values in updates:
                    values["timestamp"] = now
                    values["source"] = "PX4_SHELL_ESC_STATUS"
                    self.rx_queue.put(("motor", (motor_index, values)))

        throttle = self._parse_shell_throttle(self._shell_buffer)
        if throttle:
            key, updates = throttle
            if key != self._last_throttle_key:
                self._last_throttle_key = key
                now = time.time()
                for motor_index, pct in updates:
                    self.rx_queue.put(("throttle", (motor_index, pct, now, "PX4_SHELL_THROTTLE")))

    def _parse_shell_esc_status(self, text: str) -> Optional[Tuple[Any, List[Tuple[int, Dict[str, Any]]]]]:
        start = text.rfind("TOPIC: esc_status")
        if start < 0:
            start = text.rfind("\nesc_status")
        if start < 0:
            return None
        sample = text[start:]

        # The MAVLink shell sends the listener output in 70-byte chunks. Do NOT
        # parse while the block is still arriving, otherwise Motor 1 gets marked
        # connected from its timestamp but rpm/current/temp are still missing and
        # the later complete block has the same counter/timestamp key.
        if "\nnsh>" not in sample:
            return None

        counter = self._re_num(sample, r"\bcounter:\s*([-+0-9.]+)")
        esc_count = self._re_num(sample, r"\besc_count:\s*([-+0-9.]+)")

        section_re = re.compile(
            r"esc\[(\d+)\]\s*\(esc_report\):(?P<body>.*?)(?=\n\s*esc\[\d+\]\s*\(esc_report\):|\nnsh>|\Z)",
            re.S,
        )

        updates: List[Tuple[int, Dict[str, Any]]] = []
        timestamps = []
        for m in section_re.finditer(sample):
            esc_array_index = int(m.group(1))
            body = m.group("body")
            ts = self._re_num(body, r"\btimestamp:\s*([-+0-9.]+)")
            if ts is None or ts <= 0:
                continue

            rpm = self._re_num(body, r"\besc_rpm:\s*([-+0-9.]+)")
            voltage = self._re_num(body, r"\besc_voltage:\s*([-+0-9.]+)")
            current = self._re_num(body, r"\besc_current:\s*([-+0-9.]+)")
            temperature = self._re_num(body, r"\besc_temperature:\s*([-+0-9.]+)")
            power = self._re_num(body, r"\besc_power:\s*([-+0-9.]+)")
            address = self._re_int(body, r"\besc_address:\s*([-+0-9.]+)")
            func = self._re_num(body, r"\bactuator_function:\s*([-+0-9.]+)")

            # A real connected report from your log has all of these fields.
            # Incomplete chunks are ignored until the full report arrives.
            if rpm is None or current is None or temperature is None or func is None:
                continue

            timestamps.append((esc_array_index, int(ts)))
            motor_index = esc_array_index
            if 101 <= int(func) <= 112:
                motor_index = int(func) - 101

            values = {
                "rpm": rpm,
                "voltage_v": self._voltage_to_v(voltage),
                "current_a": self._current_to_a(current),
                "temperature_c": self._temp_to_c(temperature),
                "throttle_pct": self._power_to_pct(power),
                "esc_address": address,
                "actuator_function": int(func),
            }
            updates.append((motor_index, values))

        if not updates:
            return None

        # Include the parsed values in the key, not only timestamp/counter, so a
        # complete block can replace an earlier partial/empty one safely.
        value_key = tuple(
            (idx,
             vals.get("rpm"),
             vals.get("current_a"),
             vals.get("temperature_c"),
             vals.get("throttle_pct"),
             vals.get("actuator_function"))
            for idx, vals in updates
        )
        key = (
            int(counter) if counter is not None else None,
            int(esc_count) if esc_count is not None else None,
            tuple(timestamps),
            value_key,
        )
        return key, updates

    def _parse_shell_throttle(self, text: str) -> Optional[Tuple[Any, List[Tuple[int, float]]]]:
        # Preferred PX4 topic: actuator_motors, whose control[12] values are normalized [-1, 1].
        candidates = []
        for topic in ("actuator_motors", "actuator_outputs"):
            start = text.rfind(f"TOPIC: {topic}")
            if start >= 0:
                candidates.append((start, topic))
        if not candidates:
            return None
        start, topic = max(candidates, key=lambda x: x[0])
        sample = text[start:]

        timestamp = self._re_num(sample, r"\btimestamp:\s*([-+0-9.]+)")
        pairs: List[Tuple[int, float]] = []

        # listener can print either control[0]: value or control: [values...]
        array_name = "control" if topic == "actuator_motors" else "output"

        for m in re.finditer(rf"\b{array_name}\[(\d+)\]\s*:\s*([-+0-9.eEnNaA]+)", sample):
            idx = int(m.group(1))
            pct = self._norm_to_pct(m.group(2))
            if pct is not None:
                pairs.append((idx, pct))

        if not pairs:
            m = re.search(rf"\b{array_name}\s*:\s*\[?([^\]\n]+)\]?", sample)
            if m:
                vals = re.findall(r"[-+]?(?:nan|inf|\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", m.group(1), flags=re.I)
                for idx, raw in enumerate(vals):
                    pct = self._norm_to_pct(raw)
                    if pct is not None:
                        pairs.append((idx, pct))

        if not pairs:
            return None
        return (topic, int(timestamp) if timestamp is not None else time.time(), tuple(pairs)), pairs

    @staticmethod
    def _norm_to_pct(raw: Any) -> Optional[float]:
        f = safe_float(raw)
        if f is None:
            return None
        # actuator_outputs might be PWM us; actuator_motors is normalized.
        if 900 <= f <= 2200:
            return max(0.0, min(100.0, (f - 1000.0) / 1000.0 * 100.0))
        if f < 0:
            return 0.0
        return max(0.0, min(100.0, f * 100.0))

    @staticmethod
    def _re_num(text: str, pattern: str) -> Optional[float]:
        m = re.search(pattern, text)
        return safe_float(m.group(1)) if m else None

    @staticmethod
    def _re_int(text: str, pattern: str) -> Optional[int]:
        f = MavLinkWorker._re_num(text, pattern)
        return int(f) if f is not None else None

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
        f = MavLinkWorker._list_num(values, i)
        return int(f) if f is not None else None

    @staticmethod
    def _voltage_to_v(v: Optional[float]) -> Optional[float]:
        if v is None:
            return None
        # PX4 ESC_STATUS voltage is V; some messages use mV.
        if abs(v) > 300:
            return v / 1000.0
        return v

    @staticmethod
    def _current_to_a(v: Optional[float]) -> Optional[float]:
        return v

    @staticmethod
    def _temp_to_c(v: Optional[float]) -> Optional[float]:
        if v is None:
            return None
        if v in (255, 32767, -32768):
            return None
        # DroneCAN ESC status uses Kelvin. PX4 listener already shows Celsius.
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

    def _send_command_long(self, command: int, params: Iterable[float]) -> None:
        with self._lock:
            m = self.master
        if m is None:
            raise RuntimeError("MAVLink not connected")
        p = list(params) + [0.0] * 7
        m.mav.command_long_send(
            self.target_system,
            self.target_component,
            int(command),
            0,
            float(p[0]), float(p[1]), float(p[2]), float(p[3]),
            float(p[4]), float(p[5]), float(p[6]),
        )

    def send_px4_shell_command(self, command: str) -> None:
        with self._lock:
            m = self.master
        if m is None:
            raise RuntimeError("MAVLink not connected")
        if mavutil is None:
            raise RuntimeError("pymavlink not available")

        dev_shell = getattr(mavutil.mavlink, "SERIAL_CONTROL_DEV_SHELL", SERIAL_CONTROL_DEV_SHELL_FALLBACK)
        flag_respond = getattr(mavutil.mavlink, "SERIAL_CONTROL_FLAG_RESPOND", SERIAL_CONTROL_FLAG_RESPOND_FALLBACK)

        raw = (command.rstrip() + "\n").encode("utf-8", errors="ignore")[:70]
        data = list(raw) + [0] * (70 - len(raw))
        m.mav.serial_control_send(dev_shell, flag_respond, 0, 0, len(raw), data)

    def arm(self, force: bool) -> None:
        cfg = self.config_getter()
        if cfg.use_shell_for_arm:
            self.send_px4_shell_command("commander arm -f" if force else "commander arm")
            return
        self._send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, [1, 21196 if force else 0, 0, 0, 0, 0, 0])

    def disarm(self, force: bool) -> None:
        cfg = self.config_getter()
        if cfg.use_shell_for_arm:
            self.send_px4_shell_command("commander disarm -f" if force else "commander disarm")
            return
        self._send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, [0, 21196 if force else 0, 0, 0, 0, 0, 0])

    def motor_test(self, motor_number_1_based: int, throttle_pct: float, duration_s: float) -> None:
        cfg = self.config_getter()
        if cfg.prefer_shell_actuator_test:
            self.shell_actuator_test(motor_number_1_based, throttle_pct, duration_s)
            return

        throttle_type_percent = getattr(mavutil.mavlink, "MOTOR_TEST_THROTTLE_PERCENT", 0) if mavutil else 0
        self._send_command_long(MAV_CMD_DO_MOTOR_TEST, [motor_number_1_based, throttle_type_percent, throttle_pct, duration_s, 1, 0, 0])

    def shell_actuator_test(self, motor_number_1_based: int, throttle_pct: float, duration_s: float) -> None:
        value = max(0.0, min(1.0, float(throttle_pct) / 100.0))
        motor = max(1, min(12, int(motor_number_1_based)))
        duration = max(0.05, min(60.0, float(duration_s)))
        self.send_px4_shell_command(f"actuator_test set -m {motor} -v {value:.3f} -t {duration:.2f}")

    def stop_motor(self, motor_number_1_based: int) -> None:
        self.shell_actuator_test(motor_number_1_based, 0.0, 0.2)

    def set_servo_test_param(self) -> None:
        # Best effort. Works only if MAVLink exposes DroneCAN param bridge.
        with self._lock:
            m = self.master
        if m is None:
            raise RuntimeError("MAVLink not connected")
        param_type = getattr(mavutil.mavlink, "MAV_PARAM_EXT_TYPE_INT32", 6)
        m.mav.param_ext_set_send(
            self.target_system,
            self.target_component,
            b"SERVO_TEST",
            b"1",
            int(param_type),
        )


class RpmGauge(tk.Canvas):
    def __init__(self, parent, cfg_getter, **kwargs):
        super().__init__(parent, height=130, bg="#111827", highlightthickness=0, **kwargs)
        self.cfg_getter = cfg_getter
        self.value: Optional[float] = None
        self.overlay: Optional[Tuple[str, str]] = None
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_value(self, value: Optional[float], overlay: Optional[Tuple[str, str]]) -> None:
        self.value = value
        self.overlay = overlay
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        cfg = self.cfg_getter()
        w = max(20, self.winfo_width())
        h = max(20, self.winfo_height())
        cx = w / 2
        cy = h - 14
        r = min(w / 2 - 18, h - 28)
        if r < 20:
            return

        def angle(v: float) -> float:
            lo, hi = cfg.rpm_min, max(cfg.rpm_min + 1.0, cfg.rpm_max)
            t = max(0.0, min(1.0, (v - lo) / (hi - lo)))
            return 210.0 - 240.0 * t

        self.create_arc(cx - r, cy - r, cx + r, cy + r, start=210, extent=-240, style="arc", width=16, outline="#374151")
        a1 = angle(cfg.rpm_green_min)
        a2 = angle(cfg.rpm_green_max)
        self.create_arc(cx - r, cy - r, cx + r, cy + r, start=a1, extent=(a2 - a1), style="arc", width=16, outline="#22c55e")

        for k in range(6):
            v = cfg.rpm_min + (cfg.rpm_max - cfg.rpm_min) * k / 5
            ar = math.radians(angle(v))
            self.create_line(cx + (r - 12) * math.cos(ar), cy - (r - 12) * math.sin(ar),
                             cx + r * math.cos(ar), cy - r * math.sin(ar),
                             fill="#cbd5e1", width=2)
            self.create_text(cx + (r - 33) * math.cos(ar), cy - (r - 33) * math.sin(ar),
                             text=f"{int(v/1000)}k" if v >= 1000 else str(int(v)),
                             fill="#cbd5e1", font=("TkDefaultFont", 8))

        v = 0.0 if self.value is None else max(cfg.rpm_min, min(cfg.rpm_max, self.value))
        ar = math.radians(angle(v))
        self.create_line(cx, cy, cx + (r - 30) * math.cos(ar), cy - (r - 30) * math.sin(ar),
                         fill="#f8fafc", width=4, capstyle=tk.ROUND)
        self.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#f8fafc", outline="")
        self.create_text(cx, cy - 34, text="---" if self.value is None else f"{self.value:.0f}", fill="#f8fafc", font=("TkDefaultFont", 18, "bold"))
        self.create_text(cx, cy - 12, text="RPM", fill="#94a3b8", font=("TkDefaultFont", 10))

        if self.overlay:
            text, kind = self.overlay
            color = {"alert": "#dc2626", "ready": "#2563eb", "disconnected": "#7f1d1d"}.get(kind, "#334155")
            self.create_rectangle(8, 10, w - 8, 40, fill=color, outline="")
            self.create_text(w / 2, 25, text=text, fill="white", font=("TkDefaultFont", 10, "bold"))


class BarGauge(ttk.Frame):
    def __init__(self, parent, label: str, unit: str, max_getter, alarm_getter=None):
        super().__init__(parent)
        self.label = label
        self.unit = unit
        self.max_getter = max_getter
        self.alarm_getter = alarm_getter
        self.value: Optional[float] = None
        self.title = ttk.Label(self, text=f"{label}: --- {unit}")
        self.title.pack(anchor="w")
        self.canvas = tk.Canvas(self, height=18, bg="#111827", highlightthickness=0)
        self.canvas.pack(fill="x", expand=True)
        self.canvas.bind("<Configure>", lambda _e: self.redraw())

    def set_value(self, value: Optional[float]) -> None:
        self.value = value
        if value is None:
            self.title.configure(text=f"{self.label}: --- {self.unit}")
        else:
            txt = f"{value:.3f}" if abs(value) < 0.01 else (f"{value:.2f}" if abs(value) < 100 else f"{value:.0f}")
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
        super().__init__(parent)
        self.value: Optional[float] = None
        self.title = ttk.Label(self, text="Throttle: --- %")
        self.title.pack(anchor="w")
        self.canvas = tk.Canvas(self, height=26, bg="#111827", highlightthickness=0)
        self.canvas.pack(fill="x", expand=True)
        self.canvas.bind("<Configure>", lambda _e: self.redraw())

    def set_value(self, value: Optional[float], override: bool = False) -> None:
        self.value = value
        mode = "OVERRIDE" if override else "PX4"
        self.title.configure(text=f"Throttle ({mode}): {'---' if value is None else f'{value:.0f}'} %")
        self.redraw()

    def redraw(self) -> None:
        self.canvas.delete("all")
        w = max(4, self.canvas.winfo_width())
        h = max(4, self.canvas.winfo_height())
        self.canvas.create_rectangle(0, 0, w, h, fill="#1f2937", outline="#374151")
        if self.value is None:
            return
        frac = max(0.0, min(1.0, self.value / 100.0))
        fill = "#3b82f6"
        self.canvas.create_rectangle(2, 2, 2 + frac * (w - 4), h - 2, fill=fill, outline="")
        self.canvas.create_text(w / 2, h / 2, text=f"{self.value:.0f}%", fill="white", font=("TkDefaultFont", 10, "bold"))


class MotorPanel(ttk.Frame):
    def __init__(self, parent, idx: int, cfg_getter, override_change_cb):
        super().__init__(parent, padding=6, style="Motor.TFrame")
        self.idx = idx
        self.cfg_getter = cfg_getter
        self.override_change_cb = override_change_cb
        self.telemetry = MotorTelemetry()
        self.override_var = tk.BooleanVar(value=False)
        self.slider_var = tk.DoubleVar(value=0.0)

        header = ttk.Frame(self, style="Motor.TFrame")
        header.pack(fill="x")
        ttk.Label(header, text=f"Motor {idx+1}", font=("TkDefaultFont", 13, "bold"), style="Motor.TLabel").pack(side="left")
        self.source_label = ttk.Label(header, text="never", style="MotorDim.TLabel")
        self.source_label.pack(side="right")

        self.rpm = RpmGauge(self, cfg_getter)
        self.rpm.pack(fill="both", expand=True, pady=(4, 4))

        self.temp = BarGauge(self, "Temp", "°C", lambda: self.cfg_getter().temp_bar_max_c, lambda: self.cfg_getter().temp_alarm_c)
        self.volt = BarGauge(self, "ESC V", "V", lambda: self.cfg_getter().voltage_bar_max_v)
        self.curr = BarGauge(self, "Current", "A", lambda: self.cfg_getter().current_bar_max_a)
        for g in (self.temp, self.volt, self.curr):
            g.pack(fill="x", pady=1)

        self.throttle = ThrottleGauge(self)
        self.throttle.pack(fill="x", pady=(4, 2))

        ctrl = ttk.Frame(self, style="Motor.TFrame")
        ctrl.pack(fill="x", pady=(2, 0))
        self.override_check = ttk.Checkbutton(ctrl, text="Override", variable=self.override_var, command=self._override_changed)
        self.override_check.pack(side="left")
        self.slider = ttk.Scale(ctrl, from_=0, to=100, variable=self.slider_var, orient="horizontal", command=lambda _v: self._slider_changed())
        self.slider.pack(side="left", fill="x", expand=True, padx=6)
        self.slider_label = ttk.Label(ctrl, text="0%", width=5, style="Motor.TLabel")
        self.slider_label.pack(side="left")
        self.slider.state(["disabled"])

    def _override_changed(self) -> None:
        if self.override_var.get():
            self.slider.state(["!disabled"])
            if self.telemetry.throttle_pct is not None:
                self.slider_var.set(max(0.0, min(100.0, self.telemetry.throttle_pct)))
        else:
            self.slider.state(["disabled"])
        self.override_change_cb(self.idx)

    def _slider_changed(self) -> None:
        self.slider_label.configure(text=f"{self.slider_var.get():.0f}%")
        self.override_change_cb(self.idx)

    def override_enabled(self) -> bool:
        return bool(self.override_var.get())

    def override_value(self) -> float:
        return float(self.slider_var.get())

    def update_display(self, armed: bool, global_voltage: Optional[float]) -> Tuple[bool, bool]:
        cfg = self.cfg_getter()
        now = time.time()
        stale = (not self.telemetry.last_update) or ((now - self.telemetry.last_update) > cfg.stale_timeout_s)
        running = (self.telemetry.rpm or 0.0) >= cfg.rpm_running_threshold

        overlay = None
        if stale:
            overlay = ("DISCONNECTED", "disconnected")
        elif self.telemetry.temp_alert_active:
            overlay = ("TEMP ALERT", "alert")
        elif armed and not running:
            overlay = ("READY TO PRIME", "ready")

        self.rpm.set_value(None if stale else self.telemetry.rpm, overlay)
        self.temp.set_value(None if stale else self.telemetry.temperature_c)

        voltage = self.telemetry.voltage_v
        if (voltage is None or voltage == 0.0) and cfg.fill_missing_esc_voltage_from_battery:
            voltage = global_voltage
        self.volt.set_value(None if stale else voltage)
        self.curr.set_value(None if stale else self.telemetry.current_a)

        if not self.override_var.get():
            self.slider.state(["disabled"])
            if self.telemetry.throttle_pct is not None:
                self.slider_var.set(max(0.0, min(100.0, self.telemetry.throttle_pct)))
        else:
            self.slider.state(["!disabled"])

        shown_thr = self.override_value() if self.override_var.get() else self.telemetry.throttle_pct
        self.throttle.set_value(None if stale and shown_thr is None else shown_thr, self.override_var.get())
        self.slider_label.configure(text=f"{self.slider_var.get():.0f}%")

        age = "stale" if stale else f"{now - self.telemetry.last_update:.1f}s"
        extra = ""
        if self.telemetry.esc_address is not None:
            extra = f" addr={self.telemetry.esc_address}"
        self.source_label.configure(text=f"{self.telemetry.last_msg}{extra} · {age}")
        return stale, running


class TestsWindow(tk.Toplevel):
    def __init__(self, app: "DashboardApp"):
        super().__init__(app.root)
        self.app = app
        self.title("Tests / Sweeps")
        self.geometry("520x440")
        self.withdraw()
        self.protocol("WM_DELETE_WINDOW", self.hide)

        box = ttk.LabelFrame(self, text="Single Motor Test", padding=10)
        box.pack(fill="x", padx=10, pady=10)
        self.motor_var = tk.IntVar(value=1)
        self.pct_var = tk.DoubleVar(value=5.0)
        self.dur_var = tk.DoubleVar(value=2.0)
        self._row(box, "Motor #", self.motor_var, 0)
        self._row(box, "Throttle %", self.pct_var, 1)
        self._row(box, "Duration s", self.dur_var, 2)
        ttk.Button(box, text="Run Motor Test", command=self.run_test).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Button(box, text="Stop All Motors", command=self.stop_all).grid(row=4, column=0, columnspan=2, sticky="ew", pady=2)

        sweep = ttk.LabelFrame(self, text="Sweep", padding=10)
        sweep.pack(fill="x", padx=10, pady=10)
        self.sweep_motor_var = tk.IntVar(value=1)
        self.sweep_min_var = tk.DoubleVar(value=0.0)
        self.sweep_max_var = tk.DoubleVar(value=20.0)
        self.sweep_step_var = tk.DoubleVar(value=2.0)
        self.sweep_dt_var = tk.DoubleVar(value=0.25)
        self._row(sweep, "Motor #", self.sweep_motor_var, 0)
        self._row(sweep, "Min %", self.sweep_min_var, 1)
        self._row(sweep, "Max %", self.sweep_max_var, 2)
        self._row(sweep, "Step %", self.sweep_step_var, 3)
        self._row(sweep, "Step time s", self.sweep_dt_var, 4)
        ttk.Button(sweep, text="Start Sweep", command=self.start_sweep).grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 2))

        servo = ttk.LabelFrame(self, text="DroneCAN Servo Sweep", padding=10)
        servo.pack(fill="x", padx=10, pady=10)
        ttk.Button(servo, text="Request SERVO_TEST=1", command=self.servo_test).pack(fill="x")
        ttk.Label(servo, text="This only works if PX4 exposes DroneCAN node parameters over MAVLink. Otherwise use QGC DroneCAN parameters.", wraplength=470, foreground="#64748b").pack(anchor="w", pady=(4, 0))

    def _row(self, parent, label, var, row):
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=var).grid(row=row, column=1, sticky="ew", pady=2, padx=(8, 0))

    def show(self):
        self.deiconify()
        self.lift()

    def hide(self):
        self.withdraw()

    def run_test(self):
        try:
            self.app.worker.motor_test(int(self.motor_var.get()), float(self.pct_var.get()), float(self.dur_var.get()))
            self.app.log(f"Motor test: motor {self.motor_var.get()}, {self.pct_var.get():.1f}%, {self.dur_var.get():.1f}s")
        except Exception as exc:
            self.app.log(f"Motor test failed: {exc}")

    def stop_all(self):
        for i in range(1, self.app.cfg.motor_count + 1):
            try:
                self.app.worker.stop_motor(i)
            except Exception as exc:
                self.app.log(f"Stop motor {i} failed: {exc}")
        self.app.log("Stop command sent to all motors")

    def start_sweep(self):
        motor = int(self.sweep_motor_var.get())
        mn = float(self.sweep_min_var.get())
        mx = float(self.sweep_max_var.get())
        step = max(0.1, abs(float(self.sweep_step_var.get())))
        dt = max(0.05, float(self.sweep_dt_var.get()))
        values = []
        v = mn
        while v <= mx + 1e-9:
            values.append(v)
            v += step
        values += list(reversed(values))
        self.app.log(f"Sweep start: motor {motor}, {mn:.1f}% -> {mx:.1f}%")
        self._run_sweep_values(motor, values, dt, 0)

    def _run_sweep_values(self, motor: int, values: List[float], dt: float, index: int):
        if index >= len(values):
            self.app.log("Sweep complete")
            return
        try:
            self.app.worker.motor_test(motor, values[index], dt * 1.5)
        except Exception as exc:
            self.app.log(f"Sweep command failed: {exc}")
            return
        self.after(int(dt * 1000), lambda: self._run_sweep_values(motor, values, dt, index + 1))

    def servo_test(self):
        try:
            self.app.worker.set_servo_test_param()
            self.app.log("Requested SERVO_TEST=1")
        except Exception as exc:
            self.app.log(f"SERVO_TEST request failed: {exc}")


class DashboardApp:
    def __init__(self, root: tk.Tk, cli_connection: Optional[str] = None):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1580x980")
        self.root.minsize(1180, 720)

        self.cfg = self._load_config()
        if cli_connection:
            self.cfg.connection = cli_connection

        self._setup_style()

        self.motors = [MotorTelemetry() for _ in range(self.cfg.motor_count)]
        self.rx_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self.connected = False
        self.armed = False
        self.last_heartbeat = 0.0
        self.global_voltage: Optional[float] = None
        self.global_current: Optional[float] = None
        self.csv_file = None
        self.csv_writer = None
        self._last_override_send = [0.0] * self.cfg.motor_count
        self._pending_override_send = set()

        self._build_ui()
        self.tests_window = TestsWindow(self)
        self.worker = MavLinkWorker(self.rx_queue, self.log_threadsafe, lambda: self.cfg)
        self.worker.start()

        self._poll_queue()
        self._ui_tick()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

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
        style.configure("TCheckbutton", background="#0f172a", foreground="#e5e7eb")
        style.configure("TButton", padding=6)
        style.configure("Danger.TButton", foreground="white", background="#dc2626")
        style.map("Danger.TButton", background=[("active", "#991b1b")])

    def _build_ui(self) -> None:
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)

        self.connection_var = tk.StringVar(value=self.cfg.connection)
        self.force_arm_var = tk.BooleanVar(value=True)
        self.esc_poll_hz_var = tk.DoubleVar(value=self.cfg.esc_shell_poll_hz)
        self.throttle_poll_hz_var = tk.DoubleVar(value=self.cfg.throttle_shell_poll_hz)
        self.heartbeat_hz_var = tk.DoubleVar(value=self.cfg.heartbeat_request_hz)
        self.stale_timeout_var = tk.DoubleVar(value=self.cfg.stale_timeout_s)
        self.temp_alarm_var = tk.DoubleVar(value=self.cfg.temp_alarm_c)
        self.rpm_green_min_var = tk.DoubleVar(value=self.cfg.rpm_green_min)
        self.rpm_green_max_var = tk.DoubleVar(value=self.cfg.rpm_green_max)
        self.rpm_max_var = tk.DoubleVar(value=self.cfg.rpm_max)
        self.fill_voltage_var = tk.BooleanVar(value=self.cfg.fill_missing_esc_voltage_from_battery)
        self.poll_esc_shell_var = tk.BooleanVar(value=self.cfg.poll_esc_status_shell)
        self.poll_throttle_shell_var = tk.BooleanVar(value=self.cfg.poll_actuator_motors_shell)
        self.parse_mav_esc_var = tk.BooleanVar(value=self.cfg.parse_mavlink_esc_status)
        self.use_shell_arm_var = tk.BooleanVar(value=self.cfg.use_shell_for_arm)
        self.use_shell_test_var = tk.BooleanVar(value=self.cfg.prefer_shell_actuator_test)
        self.override_hz_var = tk.DoubleVar(value=self.cfg.override_send_hz)
        self.actuator_topic_var = tk.StringVar(value=self.cfg.actuator_motors_topic)
        self.csv_var = tk.BooleanVar(value=self.cfg.log_to_csv)

        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="MAVProxy/PX4:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.connection_var).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(top, text="Reconnect", command=self.reconnect).grid(row=0, column=2, padx=3)
        ttk.Checkbutton(top, text="Force arm", variable=self.force_arm_var).grid(row=0, column=3, padx=5)
        self.arm_button = ttk.Button(top, text="ARM", command=self.toggle_arm, style="Danger.TButton")
        self.arm_button.grid(row=0, column=4, padx=3)
        ttk.Button(top, text="Tests / Sweeps", command=lambda: self.tests_window.show()).grid(row=0, column=5, padx=3)
        ttk.Button(top, text="Settings", command=self.show_settings).grid(row=0, column=6, padx=3)
        ttk.Button(top, text="Save Config", command=self.save_config).grid(row=0, column=7, padx=3)

        body = ttk.Frame(root, padding=(8, 0, 8, 8))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=5)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        main = ttk.Frame(body)
        main.grid(row=0, column=0, sticky="nsew")
        for c in range(3):
            main.columnconfigure(c, weight=1)
        for r in range(2):
            main.rowconfigure(r, weight=1)

        self.motor_panels: List[MotorPanel] = []
        for i in range(self.cfg.motor_count):
            p = MotorPanel(main, i, self.get_config_from_ui, self._override_changed)
            p.grid(row=i // 3, column=i % 3, sticky="nsew", padx=5, pady=5)
            self.motor_panels.append(p)

        side = ttk.Frame(body)
        side.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        side.columnconfigure(0, weight=1)
        side.rowconfigure(3, weight=1)

        self.status_frame = tk.Frame(side, bg="#dc2626", padx=10, pady=10)
        self.status_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.status_label = tk.Label(self.status_frame, text="DISARMED", bg="#dc2626", fg="white", font=("TkDefaultFont", 24, "bold"))
        self.status_label.pack(fill="x")
        self.connection_label = tk.Label(self.status_frame, text="waiting for heartbeat", bg="#dc2626", fg="white")
        self.connection_label.pack(fill="x")

        quick = ttk.LabelFrame(side, text="Quick Rates", padding=8)
        quick.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        self._entry_row(quick, "ESC shell Hz", self.esc_poll_hz_var, 0)
        self._entry_row(quick, "Throttle shell Hz", self.throttle_poll_hz_var, 1)
        self._entry_row(quick, "Heartbeat request Hz", self.heartbeat_hz_var, 2)
        ttk.Button(quick, text="Apply Stream Rates Now", command=self.apply_stream_rates).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        logs = ttk.LabelFrame(side, text="Logs", padding=8)
        logs.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        ttk.Checkbutton(logs, text="CSV", variable=self.csv_var, command=self.toggle_csv).pack(side="left")
        ttk.Button(logs, text="Choose", command=self.choose_csv).pack(side="left", padx=4)
        ttk.Button(logs, text="Clear", command=lambda: self.log_text.delete("1.0", "end")).pack(side="left", padx=4)

        self.log_text = tk.Text(side, height=16, bg="#020617", fg="#e5e7eb", insertbackground="#e5e7eb", wrap="word", relief="flat")
        self.log_text.grid(row=3, column=0, sticky="nsew")
        self.log("Dashboard started")
        self.log("ESC voltage fallback from battery is OFF by default, so it will not show the misleading 65 V battery value as motor voltage.")

    def _entry_row(self, parent, label, var, row):
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=2)

    def _override_changed(self, idx: int) -> None:
        self._pending_override_send.add(idx)

    def show_settings(self):
        win = tk.Toplevel(self.root)
        win.title("Settings")
        win.geometry("560x620")
        box = ttk.LabelFrame(win, text="Polling / Sources", padding=10)
        box.pack(fill="x", padx=10, pady=10)
        self._entry_row(box, "ESC shell poll Hz", self.esc_poll_hz_var, 0)
        self._entry_row(box, "Throttle shell poll Hz", self.throttle_poll_hz_var, 1)
        self._entry_row(box, "Heartbeat request Hz", self.heartbeat_hz_var, 2)
        self._entry_row(box, "Stale timeout s", self.stale_timeout_var, 3)
        ttk.Checkbutton(box, text="Poll `listener esc_status`", variable=self.poll_esc_shell_var).grid(row=4, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Checkbutton(box, text="Poll throttle topic", variable=self.poll_throttle_shell_var).grid(row=5, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Checkbutton(box, text="Parse MAVLink ESC_STATUS too", variable=self.parse_mav_esc_var).grid(row=6, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Label(box, text="Throttle topic").grid(row=7, column=0, sticky="w", pady=2)
        ttk.Entry(box, textvariable=self.actuator_topic_var).grid(row=7, column=1, sticky="ew", padx=(8, 0), pady=2)

        gauges = ttk.LabelFrame(win, text="Gauges / Alerts", padding=10)
        gauges.pack(fill="x", padx=10, pady=10)
        self._entry_row(gauges, "Temp alarm °C", self.temp_alarm_var, 0)
        self._entry_row(gauges, "RPM green min", self.rpm_green_min_var, 1)
        self._entry_row(gauges, "RPM green max", self.rpm_green_max_var, 2)
        self._entry_row(gauges, "RPM max", self.rpm_max_var, 3)
        ttk.Checkbutton(gauges, text="Use battery voltage if ESC voltage is missing/0", variable=self.fill_voltage_var).grid(row=4, column=0, columnspan=2, sticky="w", pady=4)

        control = ttk.LabelFrame(win, text="Control", padding=10)
        control.pack(fill="x", padx=10, pady=10)
        self._entry_row(control, "Override send Hz", self.override_hz_var, 0)
        ttk.Checkbutton(control, text="Use PX4 shell for arm/disarm", variable=self.use_shell_arm_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Checkbutton(control, text="Use PX4 shell actuator_test for motor tests/override", variable=self.use_shell_test_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=4)

        buttons = ttk.Frame(win, padding=10)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="Apply Stream Rates Now", command=self.apply_stream_rates).pack(side="left")
        ttk.Button(buttons, text="Save", command=self.save_config).pack(side="right", padx=4)
        ttk.Button(buttons, text="Close", command=win.destroy).pack(side="right")

    def get_config_from_ui(self) -> DashboardConfig:
        try:
            self.cfg.connection = self.connection_var.get().strip() or self.cfg.connection
            self.cfg.esc_shell_poll_hz = float(self.esc_poll_hz_var.get())
            self.cfg.throttle_shell_poll_hz = float(self.throttle_poll_hz_var.get())
            self.cfg.heartbeat_request_hz = float(self.heartbeat_hz_var.get())
            self.cfg.stale_timeout_s = float(self.stale_timeout_var.get())
            self.cfg.temp_alarm_c = float(self.temp_alarm_var.get())
            self.cfg.rpm_green_min = float(self.rpm_green_min_var.get())
            self.cfg.rpm_green_max = float(self.rpm_green_max_var.get())
            self.cfg.rpm_max = float(self.rpm_max_var.get())
            self.cfg.fill_missing_esc_voltage_from_battery = bool(self.fill_voltage_var.get())
            self.cfg.poll_esc_status_shell = bool(self.poll_esc_shell_var.get())
            self.cfg.poll_actuator_motors_shell = bool(self.poll_throttle_shell_var.get())
            self.cfg.parse_mavlink_esc_status = bool(self.parse_mav_esc_var.get())
            self.cfg.use_shell_for_arm = bool(self.use_shell_arm_var.get())
            self.cfg.prefer_shell_actuator_test = bool(self.use_shell_test_var.get())
            self.cfg.override_send_hz = float(self.override_hz_var.get())
            self.cfg.actuator_motors_topic = self.actuator_topic_var.get().strip() or "actuator_motors"
            self.cfg.log_to_csv = bool(self.csv_var.get())
        except Exception:
            pass
        return self.cfg

    def _load_config(self) -> DashboardConfig:
        cfg = DashboardConfig()
        if os.path.exists(CONFIG_FILE):
            try:
                data = json.loads(Path(CONFIG_FILE).read_text())
                for k, v in data.items():
                    if hasattr(cfg, k):
                        setattr(cfg, k, v)
            except Exception:
                pass
        # Keep the new safety default even if an old config had fallback enabled.
        if not hasattr(cfg, "fill_missing_esc_voltage_from_battery"):
            cfg.fill_missing_esc_voltage_from_battery = False
        return cfg

    def save_config(self):
        self.get_config_from_ui()
        try:
            Path(CONFIG_FILE).write_text(json.dumps(asdict(self.cfg), indent=2))
            self.log(f"Saved config to {CONFIG_FILE}")
        except Exception as exc:
            self.log(f"Save config failed: {exc}")

    def reconnect(self):
        self.get_config_from_ui()
        try:
            self.worker.close()
        except Exception:
            pass
        self.worker = MavLinkWorker(self.rx_queue, self.log_threadsafe, lambda: self.cfg)
        self.worker.start()
        self.log("Reconnect requested")

    def toggle_arm(self):
        force = bool(self.force_arm_var.get())
        try:
            if self.armed:
                self.worker.disarm(force=force)
                self.log("Disarm command sent" + (" through shell -f" if force else ""))
            else:
                self.worker.arm(force=force)
                self.log("Arm command sent" + (" through shell -f" if force else ""))
        except Exception as exc:
            self.log(f"Arm/disarm failed: {exc}")

    def apply_stream_rates(self):
        self.get_config_from_ui()
        try:
            if self.cfg.heartbeat_request_hz > 0:
                self.worker.request_message_interval_now(MAVLINK_MSG_ID_HEARTBEAT, self.cfg.heartbeat_request_hz)
            if self.cfg.esc_mavlink_request_hz > 0:
                self.worker.request_message_interval_now(MAVLINK_MSG_ID_ESC_STATUS, self.cfg.esc_mavlink_request_hz)
                self.worker.request_message_interval_now(MAVLINK_MSG_ID_ESC_INFO, min(5.0, self.cfg.esc_mavlink_request_hz))
            self.log("Applied stream-rate requests")
        except Exception as exc:
            self.log(f"Apply stream rates failed: {exc}")

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
                elif kind == "ack":
                    txt = self._format_ack(payload)
                    if txt:
                        self.log(txt)
                elif kind == "error":
                    self.log(f"ERROR: {payload}")
                elif kind == "log":
                    self.log(str(payload))
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _handle_motor_update(self, idx: int, values: Dict[str, Any]):
        if idx < 0 or idx >= len(self.motors):
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
        m.stale_announced = False
        self._check_alerts(idx, m)
        self._write_csv(idx, m)

    def _handle_throttle_update(self, idx: int, pct: float, timestamp: float, source: str):
        if idx < 0 or idx >= len(self.motors):
            return
        m = self.motors[idx]
        m.throttle_pct = float(pct)
        # Do not overwrite last_msg if ESC data is fresh; throttle is a secondary datum.
        if not m.last_update:
            m.last_update = timestamp
            m.last_msg = source

    def _check_alerts(self, idx: int, m: MotorTelemetry):
        if m.temperature_c is not None and m.temperature_c >= self.cfg.temp_alarm_c:
            if not m.temp_alert_active:
                m.temp_alert_active = True
                self.log(f"TEMP ALERT: Motor {idx+1}: {m.temperature_c:.1f} °C")
                try:
                    self.root.bell()
                except Exception:
                    pass
        else:
            m.temp_alert_active = False

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

    def _format_ack(self, d: Dict[str, Any]) -> Optional[str]:
        names = {
            MAV_CMD_COMPONENT_ARM_DISARM: "COMPONENT_ARM_DISARM",
            MAV_CMD_DO_MOTOR_TEST: "DO_MOTOR_TEST",
            MAV_CMD_SET_MESSAGE_INTERVAL: "SET_MESSAGE_INTERVAL",
            MAV_CMD_DO_SET_ACTUATOR: "DO_SET_ACTUATOR",
            410: "GET_HOME_POSITION",
        }
        results = {
            0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
            3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED",
        }
        cmd = d.get("command")
        result = d.get("result")
        try:
            cmd_i = int(cmd)
            res_i = int(result)
        except Exception:
            return f"COMMAND_ACK: {d}"

        # Hide repetitive unrelated home-position denials unless it is useful.
        if cmd_i == 410 and res_i == 2:
            return None
        return f"COMMAND_ACK: {names.get(cmd_i, cmd_i)} ({cmd_i}) -> {results.get(res_i, res_i)} ({res_i})"

    def _ui_tick(self):
        cfg = self.get_config_from_ui()
        now = time.time()
        heartbeat_age = now - self.last_heartbeat if self.last_heartbeat else 9999
        heartbeat_ok = heartbeat_age < cfg.heartbeat_timeout_s
        if not heartbeat_ok:
            self.connected = False

        stale_count = 0
        running_count = 0
        for i, panel in enumerate(self.motor_panels):
            panel.telemetry = self.motors[i]
            stale, running = panel.update_display(self.armed, self.global_voltage)
            stale_count += int(stale)
            running_count += int(running and not stale)
            if stale and self.motors[i].last_update and not self.motors[i].stale_announced:
                self.motors[i].stale_announced = True
                self.log(f"Motor {i+1} stale: no telemetry for {cfg.stale_timeout_s:.1f}s")

        self._send_overrides()

        if not heartbeat_ok:
            color, status, detail = "#991b1b", "NO HEARTBEAT", f"{heartbeat_age:.1f}s since heartbeat"
            button = "ARM"
        elif not self.armed:
            color, status, detail = "#dc2626", "DISARMED", f"Connected · ESC shell {cfg.esc_shell_poll_hz:g} Hz · throttle {cfg.throttle_shell_poll_hz:g} Hz"
            button = "ARM"
        else:
            color = "#16a34a" if running_count else "#2563eb"
            status = "RUNNING" if running_count else "READY TO PRIME"
            detail = f"{running_count}/{cfg.motor_count} running · {stale_count} stale"
            button = "DISARM"

        self.status_frame.configure(bg=color)
        self.status_label.configure(text=status, bg=color)
        self.connection_label.configure(text=detail, bg=color)
        self.arm_button.configure(text=button)

        self.root.after(120, self._ui_tick)

    def _send_overrides(self):
        cfg = self.cfg
        now = time.time()
        period = 1.0 / max(0.2, cfg.override_send_hz)
        for i, panel in enumerate(self.motor_panels):
            if not panel.override_enabled():
                continue
            if (now - self._last_override_send[i]) < period and i not in self._pending_override_send:
                continue
            self._pending_override_send.discard(i)
            self._last_override_send[i] = now
            try:
                self.worker.motor_test(i + 1, panel.override_value(), cfg.actuator_test_duration_s)
            except Exception as exc:
                self.log(f"Override send failed motor {i+1}: {exc}")

    def choose_csv(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if path:
            self.cfg.csv_path = path
            self.log(f"CSV path: {path}")

    def toggle_csv(self):
        self.get_config_from_ui()
        if self.cfg.log_to_csv:
            self._open_csv()
        else:
            self._close_csv()

    def _open_csv(self):
        if self.csv_writer:
            return
        try:
            new = not os.path.exists(self.cfg.csv_path)
            self.csv_file = open(self.cfg.csv_path, "a", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            if new:
                self.csv_writer.writerow(["time", "motor", "rpm", "temp_c", "esc_v", "current_a", "throttle_pct", "source", "armed"])
            self.log(f"CSV logging ON: {self.cfg.csv_path}")
        except Exception as exc:
            self.log(f"CSV open failed: {exc}")
            self.csv_var.set(False)

    def _close_csv(self):
        try:
            if self.csv_file:
                self.csv_file.close()
        finally:
            self.csv_file = None
            self.csv_writer = None
        self.log("CSV logging OFF")

    def _write_csv(self, idx: int, m: MotorTelemetry):
        if not self.csv_writer:
            return
        try:
            self.csv_writer.writerow([
                datetime.now().isoformat(timespec="milliseconds"), idx + 1, m.rpm,
                m.temperature_c, m.voltage_v, m.current_a, m.throttle_pct, m.last_msg, int(self.armed)
            ])
            if self.csv_file:
                self.csv_file.flush()
        except Exception as exc:
            self.log(f"CSV write failed: {exc}")

    def log_threadsafe(self, msg: str):
        self.rx_queue.put(("log", msg))

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        try:
            self.log_text.insert("end", f"[{ts}] {msg}\n")
            self.log_text.see("end")
            lines = int(self.log_text.index("end-1c").split(".")[0])
            if lines > 900:
                self.log_text.delete("1.0", "120.0")
        except Exception:
            pass

    def _on_close(self):
        try:
            self.save_config()
        except Exception:
            pass
        try:
            self.worker.close()
        except Exception:
            pass
        try:
            self._close_csv()
        except Exception:
            pass
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", "--connect", dest="connection", default=None)
    args = parser.parse_args()

    root = tk.Tk()
    DashboardApp(root, cli_connection=args.connection)
    root.mainloop()


if __name__ == "__main__":
    main()
