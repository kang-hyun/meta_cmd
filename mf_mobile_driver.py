#!/usr/bin/env python3
"""MFPacket transport and command shaping for two driven wheels + passive casters.

No caster angle is measured or inferred here. The motion profile only reduces
command transients; it cannot correct body heading or lateral displacement.
"""

from collections import deque
from dataclasses import dataclass
import math
import struct
import time

import serial


@dataclass(frozen=True)
class MotionConfig:
    wheel_radius: float = 0.12
    wheel_separation: float = 0.70
    max_linear_speed: float = 0.30
    max_angular_speed: float = 0.50
    max_wheel_speed: float = 20.0
    wheel_acceleration: float = 2.0
    wheel_deceleration: float = 4.0
    startup_duration: float = 0.8
    startup_linear_speed: float = 0.08
    startup_angular_speed: float = 0.20
    reversal_pause: float = 0.20

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.startup_linear_speed > self.max_linear_speed:
            raise ValueError("startup_linear_speed must be <= max_linear_speed")
        if self.startup_angular_speed > self.max_angular_speed:
            raise ValueError("startup_angular_speed must be <= max_angular_speed")


class CasterMotionProfile:
    """Common-ratio speed caps, coupled wheel ramps, and stop-before-reverse.

    Zero commands bypass ramps. `measured_stopped` is True/False only for fresh
    feedback, or None when unavailable. With require_feedback=False, unavailable
    feedback uses the zero-command dwell only; physical standstill is unverified.
    Fresh moving feedback still blocks departure. Reset restarts the dwell.
    """

    def __init__(self, config: MotionConfig):
        self.config = config
        self.reset()

    def reset(self):
        self.left = self.right = 0.0
        self.phase = "idle"
        self._startup_at = None
        self._stopped_at = None

    def _ramp(self, left, right, dt):
        ratio = 1.0
        for current, target in ((self.left, left), (self.right, right)):
            delta = target - current
            if abs(delta) > 1e-12:
                limit = (self.config.wheel_deceleration if abs(target) < abs(current)
                         else self.config.wheel_acceleration)
                ratio = min(ratio, limit * dt / abs(delta))
        self.left += ratio * (left - self.left)
        self.right += ratio * (right - self.right)
        if abs(self.left) < 1e-10:
            self.left = 0.0
        if abs(self.right) < 1e-10:
            self.right = 0.0

    def step(self, linear, angular, now, dt, measured_stopped, require_feedback=True):
        if not all(math.isfinite(v) for v in (linear, angular, now, dt)) or dt <= 0:
            raise ValueError("invalid motion input")
        if abs(linear) < 1e-9 and abs(angular) < 1e-9:
            self.reset()
            return 0.0, 0.0
        c = self.config
        scale = min(1.0, c.max_linear_speed / max(abs(linear), 1e-12),
                    c.max_angular_speed / max(abs(angular), 1e-12))
        linear, angular = linear * scale, angular * scale
        left = (linear - angular * c.wheel_separation / 2) / c.wheel_radius
        right = (linear + angular * c.wheel_separation / 2) / c.wheel_radius
        scale = min(1.0, c.max_wheel_speed / max(abs(left), abs(right), 1e-12))
        left, right = left * scale, right * scale

        # A sign reversal of either wheel brakes both, keeping their ratio.
        if self.phase not in ("braking", "reversal_wait", "idle") and (
            self.left * left < -1e-10 or self.right * right < -1e-10
        ):
            self.phase = "braking"
            self._stopped_at = None
        if self.phase == "braking":
            self._ramp(0.0, 0.0, dt)
            if self.left == 0.0 and self.right == 0.0:
                self.phase = "reversal_wait"
            return self.left, self.right
        if self.phase in ("idle", "reversal_wait"):
            stop_ready = measured_stopped is True or (measured_stopped is None and not require_feedback)
            if not stop_ready:
                self._stopped_at = None
                return 0.0, 0.0
            if self._stopped_at is None:
                self._stopped_at = now
            if now - self._stopped_at < c.reversal_pause:
                return 0.0, 0.0
            self.phase = "startup"
            self._startup_at = now
        if self.phase == "startup":
            if now - self._startup_at < c.startup_duration:
                scale = min(1.0, c.startup_linear_speed / max(abs(linear), 1e-12),
                            c.startup_angular_speed / max(abs(angular), 1e-12))
                left, right = left * scale, right * scale
            else:
                self.phase = "running"
        self._ramp(left, right, dt)
        return self.left, self.right


class MfMobileDriver:
    _HEADER = b"\xff\xf9"
    _STATE_ID, _SPEED_ID = 0x01, 0x02
    _LIFT_STATE_ID, _LIFT_COMMAND_ID = 0x05, 0x06
    _DOUBLE, _INT = 0x02, 0x00
    LIFT_STOP, LIFT_UP, LIFT_DOWN, LIFT_HOME = range(4)
    _SIZES = {_STATE_ID: 67, _LIFT_STATE_ID: 33}

    def __init__(self, port="/dev/ttyUSB0", baudrate=921600, wheel_radius=0.12,
                 wheel_separation=0.70, max_wheel_speed=20.0, enable_lift=False):
        for name, value in (("baudrate", baudrate), ("wheel_radius", wheel_radius),
                            ("wheel_separation", wheel_separation),
                            ("max_wheel_speed", max_wheel_speed)):
            self._require_positive(name, value)
        if isinstance(baudrate, bool) or int(baudrate) != baudrate:
            raise ValueError("baudrate must be an integer")
        if not isinstance(enable_lift, bool):
            raise ValueError("enable_lift must be boolean")
        self.wheel_radius = float(wheel_radius)
        self.wheel_separation = float(wheel_separation)
        self.max_wheel_speed = float(max_wheel_speed)
        self._enable_lift = enable_lift
        self._rx_buffer = bytearray()
        self._motor_states = deque(maxlen=200)
        self._lift_states = deque(maxlen=200)
        self.bad_frames = 0
        self.rx_bytes = self.tx_bytes = self.tx_packets = 0
        self.motor_packets = self.lift_packets = 0
        self.last_rx_hex = self.last_tx_hex = ""
        self._serial = serial.Serial(port, int(baudrate), timeout=0, write_timeout=0.2)

    @staticmethod
    def _require_finite(name, value):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")

    @classmethod
    def _require_positive(cls, name, value):
        cls._require_finite(name, value)
        if value <= 0:
            raise ValueError(f"{name} must be > 0")

    @staticmethod
    def _checksum(data):
        # Preserve the firmware-specific algorithm, not a generic CRC library.
        crc = 0xFF
        for value in data:
            crc ^= value
            for _ in range(8):
                crc = (crc >> 1) ^ (0x1D if crc & 1 else 0)
        return crc & 0xFF

    @classmethod
    def _speed_packet(cls, left, right):
        body = bytearray(cls._HEADER + bytes((23, cls._SPEED_ID, 0, 2)))
        for value in (left, right):
            body.extend(bytes((cls._DOUBLE, 8)) + struct.pack("<d", value))
        body.append(cls._checksum(body))
        return bytes(body)

    @classmethod
    def _lift_packet(cls, command):
        if command not in (0, 1, 2, 3):
            raise ValueError("invalid lift command")
        body = bytearray(cls._HEADER + bytes((9, cls._LIFT_COMMAND_ID, 0, 1, cls._INT, 4)))
        body.extend(struct.pack("<i", command))
        body.append(cls._checksum(body))
        return bytes(body)

    def _write(self, packet):
        if not self.is_open:
            raise RuntimeError("serial port is closed")
        written = self._serial.write(packet)
        self.tx_bytes += written
        if written != len(packet):
            raise serial.SerialException("incomplete serial write")
        self.tx_packets += 1
        self.last_tx_hex = packet.hex(" ")

    def set_velocity(self, linear_x, angular_z):
        self._require_finite("linear_x", linear_x)
        self._require_finite("angular_z", angular_z)
        half_turn = angular_z * self.wheel_separation / 2
        left, right = ((linear_x - half_turn) / self.wheel_radius,
                       (linear_x + half_turn) / self.wheel_radius)
        self.set_wheel_speeds(left, right)
        return left, right

    def set_wheel_speeds(self, left, right):
        self._require_finite("left", left)
        self._require_finite("right", right)
        if max(abs(left), abs(right)) > self.max_wheel_speed + 1e-9:
            raise ValueError(f"wheel speed exceeds limit of {self.max_wheel_speed} rad/s")
        self._write(self._speed_packet(left, right))

    @classmethod
    def _valid_frame(cls, packet, packet_id, count):
        return (len(packet) == cls._SIZES[packet_id]
                and packet[:2] == cls._HEADER
                and packet[2] == len(packet) - 4 and packet[3] == packet_id
                and packet[4] in (0, 2, 3) and packet[5] == count
                and cls._checksum(packet[:-1]) == packet[-1])

    @classmethod
    def _parse_state(cls, packet):
        if not cls._valid_frame(packet, cls._STATE_ID, 6):
            return None
        values = []
        for offset in range(6, 66, 10):
            if packet[offset:offset + 2] != bytes((cls._DOUBLE, 8)):
                return None
            values.append(struct.unpack_from("<d", packet, offset + 2)[0])
        return tuple(values) if all(map(math.isfinite, values)) else None

    @classmethod
    def _parse_lift_state(cls, packet):
        if not cls._valid_frame(packet, cls._LIFT_STATE_ID, 3):
            return None
        if (packet[6:8] != bytes((cls._DOUBLE, 8))
                or packet[16:18] != bytes((cls._DOUBLE, 8))
                or packet[26:28] != bytes((cls._INT, 4))):
            return None
        position, adc, flags = (struct.unpack_from("<d", packet, 8)[0],
                                struct.unpack_from("<d", packet, 18)[0],
                                struct.unpack_from("<i", packet, 28)[0])
        if not math.isfinite(position) or not math.isfinite(adc) or flags < 0:
            return None
        return position, adc, flags

    def _poll(self):
        if not self.is_open:
            raise RuntimeError("serial port is closed")
        waiting = self._serial.in_waiting
        if waiting > 8192 or len(self._rx_buffer) + waiting > 8192:
            self._rx_buffer.clear()
            raise serial.SerialException("receive backlog exceeded 8192 bytes")
        if waiting:
            data = self._serial.read(waiting)
            self.rx_bytes += len(data)
            self.last_rx_hex = data[:128].hex(" ")
            self._rx_buffer.extend(data)
        while True:
            index = self._rx_buffer.find(self._HEADER)
            if index < 0:
                # Keep only a possible partial header.
                self._rx_buffer[:] = b"\xff" if self._rx_buffer[-1:] == b"\xff" else b""
                return
            if index:
                del self._rx_buffer[:index]
            if len(self._rx_buffer) < 4:
                return
            size = self._SIZES.get(self._rx_buffer[3])
            if size is None or self._rx_buffer[2] != size - 4:
                self.bad_frames += 1
                del self._rx_buffer[0]
                continue
            if len(self._rx_buffer) < size:
                return
            packet = bytes(self._rx_buffer[:size])
            if self._checksum(packet[:-1]) != packet[-1]:
                self.bad_frames += 1
                del self._rx_buffer[0]
                continue
            del self._rx_buffer[:size]
            state = self._parse_state(packet) if packet[3] == self._STATE_ID else self._parse_lift_state(packet)
            if state is None:
                self.bad_frames += 1
            elif packet[3] == self._STATE_ID:
                self.motor_packets += 1
                self._motor_states.append(state)
            else:
                self.lift_packets += 1
                if self._enable_lift:
                    self._lift_states.append(state)

    def read_states(self):
        self._poll()
        states = list(self._motor_states)
        self._motor_states.clear()
        return states

    def read_lift_states(self):
        if not self._enable_lift:
            return []
        self._poll()
        states = list(self._lift_states)
        self._lift_states.clear()
        return states

    def set_lift_command(self, command):
        if not self._enable_lift:
            raise RuntimeError("lift support is disabled")
        self._write(self._lift_packet(command))

    @property
    def lift_enabled(self):
        return self._enable_lift

    @property
    def is_open(self):
        return bool(self._serial and self._serial.is_open)

    def stop(self, repeat=5, interval=0.02):
        if not isinstance(repeat, int) or repeat < 1 or not math.isfinite(interval) or interval < 0:
            raise ValueError("invalid stop repeat/interval")
        if not self.is_open:
            return
        packets = [self._speed_packet(0.0, 0.0)]
        if self._enable_lift:
            packets.append(self._lift_packet(self.LIFT_STOP))
        first_error = None
        for index in range(repeat):
            for packet in packets:
                try:
                    self._write(packet)
                except (OSError, RuntimeError, serial.SerialException) as error:
                    first_error = first_error or error
            if interval and index + 1 < repeat:
                time.sleep(interval)
        if first_error is not None:
            raise first_error

    def close(self):
        if self.is_open:
            try:
                self.stop()
            finally:
                self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


__all__ = ["MfMobileDriver", "MotionConfig", "CasterMotionProfile"]
