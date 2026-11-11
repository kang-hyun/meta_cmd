"""Low-temperature robot transport, based on mobile_keti_261109.zip.

No ROS dependency in this module. pyserial is imported only when opening a
real port. See docs/VENDOR_PROTOCOL.md for the unchanged wire contract.
"""

import math
import struct
import time

HEADER = b"\xff\xf9"
TYPE_DOUBLE = 0x02
ACK_NO_ACK = 0x00
MOTOR_STATE_ID = 0x01
MOTOR_SPEED_ID = 0x02
MAX_FRAME_SIZE = 97  # Largest frame in the supplied protocol.


def checksum(data):
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x1D if crc & 1 else 0)
    return crc & 0xFF


def encode_doubles(packet_id, values):
    values = tuple(values)
    fields = b"".join(bytes((TYPE_DOUBLE, 8)) + struct.pack("<d", v) for v in values)
    packet = HEADER + bytes((3 + len(fields), packet_id, ACK_NO_ACK, len(values))) + fields
    return packet + bytes((checksum(packet),))


def decode_motor_state(packet):
    if (len(packet) != 67 or packet[:6] != HEADER + bytes((63, MOTOR_STATE_ID, 0, 6))
            or checksum(packet[:-1]) != packet[-1]):
        return None
    values = []
    for offset in range(6, 66, 10):
        if packet[offset:offset + 2] != bytes((TYPE_DOUBLE, 8)):
            return None
        value = struct.unpack_from("<d", packet, offset + 2)[0]
        if not math.isfinite(value):
            return None
        values.append(value)
    return tuple(values)


class StateParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0
        self.motor_packets = 0

    def feed(self, data):
        self.buffer.extend(data)
        states = []
        while True:
            index = self.buffer.find(HEADER)
            if index < 0:
                # Preserve a split header, but discard all other noise.
                self.buffer[:] = b"\xff" if self.buffer.endswith(b"\xff") else b""
                break
            del self.buffer[:index]
            if len(self.buffer) < 3:
                break
            size = self.buffer[2] + 4
            if not 7 <= size <= MAX_FRAME_SIZE:
                self.bad_frames += 1
                del self.buffer[:1]
                continue
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size])
            if checksum(packet[:-1]) != packet[-1]:
                self.bad_frames += 1
                del self.buffer[:1]
                continue
            del self.buffer[:size]
            if packet[3] == MOTOR_STATE_ID:
                state = decode_motor_state(packet)
                if state is None:
                    self.bad_frames += 1
                else:
                    self.motor_packets += 1
                    states.append(state)
        return states


def positive_finite(name, value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and > 0")
    return float(value)


class VelocityCommand:
    """Direct differential-drive conversion; no startup cap or host ramp."""

    def __init__(self, wheel_radius=0.15, wheel_separation=0.60,
                 max_wheel_speed=20.0, timeout=0.5):
        self.radius = positive_finite("wheel_radius", wheel_radius)
        self.separation = positive_finite("wheel_separation", wheel_separation)
        self.limit = positive_finite("max_wheel_speed", max_wheel_speed)
        self.timeout = positive_finite("cmd_vel_timeout", timeout)
        self.last_command = None
        self.wheels = (0.0, 0.0)
        self.reason = "waiting_cmd_vel"

    def accept(self, linear_x, angular_z, now):
        try:
            if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
                raise ValueError("cmd_vel linear.x/angular.z must be finite")
            half_turn = angular_z * self.separation / 2.0
            wheels = ((linear_x - half_turn) / self.radius,
                      (linear_x + half_turn) / self.radius)
            if not all(math.isfinite(w) and abs(w) <= self.limit for w in wheels):
                raise ValueError(f"cmd_vel exceeds max_wheel_speed={self.limit} rad/s")
        except ValueError:
            # An invalid new command must not keep an earlier moving target alive.
            self.wheels = (0.0, 0.0)
            self.last_command = None
            self.reason = "invalid_cmd_vel"
            raise
        self.wheels = wheels
        self.last_command = now
        self.reason = "cmd_vel"

    def target(self, now):
        if self.last_command is None:
            return (0.0, 0.0)
        if now - self.last_command >= self.timeout:
            self.reason = "cmd_vel_timeout"
            return (0.0, 0.0)
        return self.wheels


class LowTempDriver:
    """Serial transport only. Feedback is never a prerequisite for sending."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=500000,
                 max_wheel_speed=20.0, serial_factory=None):
        positive_finite("baudrate", baudrate)
        if int(baudrate) != baudrate:
            raise ValueError("baudrate must be an integer")
        self.max_wheel_speed = positive_finite("max_wheel_speed", max_wheel_speed)
        if serial_factory is None:
            from serial import Serial
            serial_factory = Serial
        self.parser = StateParser()
        self.tx_packets = 0
        self.rx_bytes = 0
        self._serial = serial_factory(
            port, int(baudrate), timeout=0, write_timeout=0.2, exclusive=True
        )

    def set_wheel_speeds(self, left, right):
        if not all(math.isfinite(w) and abs(w) <= self.max_wheel_speed for w in (left, right)):
            raise ValueError("wheel speed is non-finite or exceeds max_wheel_speed")
        if not self._serial.is_open:
            raise OSError("serial port is closed")
        packet = encode_doubles(MOTOR_SPEED_ID, (left, right))
        written = self._serial.write(packet)
        if written != len(packet):
            raise OSError(f"incomplete serial write: {written}/{len(packet)} bytes")
        self.tx_packets += 1

    def read_states(self):
        if not self._serial.is_open:
            raise OSError("serial port is closed")
        # Bound work per control tick; normal RX is about 3350 bytes/second.
        waiting = min(self._serial.in_waiting, 8192)
        data = self._serial.read(waiting) if waiting else b""
        self.rx_bytes += len(data)
        return self.parser.feed(data)

    def stop(self, repeat=5, interval=0.02):
        if not self._serial.is_open:
            return
        for index in range(repeat):
            self.set_wheel_speeds(0.0, 0.0)
            if interval and index + 1 < repeat:
                time.sleep(interval)

    def close(self):
        if self._serial.is_open:
            try:
                self.stop()
            finally:
                self._serial.close()
