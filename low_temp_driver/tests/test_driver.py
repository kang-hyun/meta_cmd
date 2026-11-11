"""Hardware-free protocol, transport, command, and ROS-callback tests.

ROS is replaced by small test doubles; these do not verify DDS or a ROS install.
"""

import importlib.util
import math
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

from low_temp_driver import (
    LowTempDriver, MOTOR_SPEED_ID, MOTOR_STATE_ID, StateParser, VelocityCommand,
    checksum, decode_motor_state, encode_doubles,
)

SPEED = bytes.fromhex("fff9170200020208000000000000f83f020800000000000002c011")
ZERO = bytes.fromhex("fff917020002020800000000000000000208000000000000000002")
STATE = bytes.fromhex(
    "fff93f0100060208000000000000f03f0208000000000000d03f02080000000000000c40"
    "0208000000000000f0bf0208000000000000c03f020800000000000000c006"
)
STATE_VALUES = (1.0, 0.25, 3.5, -1.0, 0.125, -2.0)


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.is_open = True
        self.rx = bytearray()
        self.writes = []
        self.write_failure = False
        self.short_write = False
        self.read_failure = False

    @property
    def in_waiting(self):
        if self.read_failure:
            raise OSError("USB disconnected")
        return len(self.rx)

    def read(self, count):
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def write(self, data):
        if self.write_failure:
            raise OSError("USB disconnected")
        self.writes.append(data)
        return len(data) - 1 if self.short_write else len(data)

    def close(self):
        self.is_open = False


class ProtocolTests(unittest.TestCase):
    def test_vendor_golden_frames(self):
        self.assertEqual(encode_doubles(MOTOR_SPEED_ID, (1.5, -2.25)), SPEED)
        self.assertEqual(encode_doubles(MOTOR_SPEED_ID, (0.0, 0.0)), ZERO)
        self.assertEqual(decode_motor_state(STATE), STATE_VALUES)

    def test_every_packet_split_including_header(self):
        for split in range(len(STATE) + 1):
            with self.subTest(split=split):
                parser = StateParser()
                self.assertEqual(parser.feed(STATE[:split]) + parser.feed(STATE[split:]),
                                 [STATE_VALUES])

    def test_byte_at_a_time_and_multiple_frames(self):
        parser = StateParser()
        result = []
        for byte in STATE * 3:
            result.extend(parser.feed(bytes((byte,))))
        self.assertEqual(result, [STATE_VALUES] * 3)
        self.assertEqual(parser.motor_packets, 3)

    def test_noise_crc_corruption_and_invalid_lengths_resync(self):
        bad = bytearray(STATE)
        bad[-1] ^= 1
        parser = StateParser()
        stream = b"boot noise\xff\xf9\x00\xff\xf9\xff" + bytes(bad) + STATE
        self.assertEqual(parser.feed(stream), [STATE_VALUES])
        self.assertGreaterEqual(parser.bad_frames, 3)

    def test_unknown_ids_and_ack_replies_are_ignored(self):
        parser = StateParser()
        self.assertEqual(parser.feed(bytes.fromhex("fff90303030013") + SPEED + STATE),
                         [STATE_VALUES])
        self.assertEqual(parser.bad_frames, 0)

    def test_malformed_but_crc_valid_state_is_rejected(self):
        for offset, value in ((2, 62), (4, 3), (5, 5), (6, 1), (7, 4)):
            with self.subTest(offset=offset):
                packet = bytearray(STATE)
                packet[offset] = value
                packet[-1] = checksum(packet[:-1])
                self.assertIsNone(decode_motor_state(packet))

    def test_nan_and_infinity_feedback_dropped_real_zero_preserved(self):
        parser = StateParser()
        for bad_value in (math.nan, math.inf, -math.inf):
            packet = encode_doubles(MOTOR_STATE_ID, (bad_value, 0, 0, 0, 0, 0))
            self.assertEqual(parser.feed(packet), [])
        self.assertEqual(parser.bad_frames, 3)
        self.assertEqual(parser.feed(encode_doubles(MOTOR_STATE_ID, (0,) * 6)), [(0,) * 6])

    def test_noise_and_incomplete_frame_memory_is_bounded(self):
        parser = StateParser()
        parser.feed(b"x" * 100000 + b"\xff")
        self.assertEqual(parser.buffer, b"\xff")
        self.assertEqual(parser.feed(STATE[1:]), [STATE_VALUES])
        parser.feed(b"\xff\xf9\x5d" + b"x" * 40)
        self.assertLess(len(parser.buffer), 97)


class CommandTests(unittest.TestCase):
    def test_waiting_initially_sends_zero(self):
        command = VelocityCommand()
        self.assertEqual(command.target(100), (0, 0))
        self.assertEqual(command.reason, "waiting_cmd_vel")

    def test_geometry_straight_turn_reverse_and_combined(self):
        command = VelocityCommand()
        for linear, angular, expected in ((0.3, 0, (2, 2)), (0, 1, (-2, 2)),
                                          (-0.3, 0, (-2, -2)), (0.3, 0.5, (1, 3))):
            command.accept(linear, angular, 1.0)
            for actual, wanted in zip(command.target(1.0), expected):
                self.assertAlmostEqual(actual, wanted)

    def test_first_command_and_direction_change_have_no_host_ramp(self):
        command = VelocityCommand()
        command.accept(1.5, 0, 1.0)
        self.assertEqual(command.target(1.0), (10, 10))
        command.accept(-1.5, 0, 1.01)
        self.assertEqual(command.target(1.01), (-10, -10))

    def test_timeout_and_new_command_recovery(self):
        command = VelocityCommand()
        command.accept(0.3, 0, 1.0)
        self.assertEqual(command.target(1.49), (2, 2))
        self.assertEqual(command.target(1.5), (0, 0))
        self.assertEqual(command.reason, "cmd_vel_timeout")
        command.accept(0.3, 0, 2.0)
        self.assertEqual(command.target(2.0), (2, 2))

    def test_zero_command_has_no_host_deceleration_delay(self):
        command = VelocityCommand()
        command.accept(1.5, 0, 0.0)
        command.accept(0, 0, 0.01)
        self.assertEqual(command.target(0.01), (0, 0))

    def test_invalid_or_over_limit_command_clears_previous_motion(self):
        for linear, angular in ((math.nan, 0), (0, math.inf), (3.01, 0),
                                (3, 1), (1e308, 1e308)):
            with self.subTest(linear=linear, angular=angular):
                command = VelocityCommand()
                command.accept(0.3, 0, 0.0)
                with self.assertRaises(ValueError):
                    command.accept(linear, angular, 0.1)
                self.assertEqual(command.target(0.1), (0, 0))
                self.assertEqual(command.reason, "invalid_cmd_vel")

    def test_parameter_validation_and_limit_boundary(self):
        for name in ("wheel_radius", "wheel_separation", "max_wheel_speed", "timeout"):
            for value in (0, -1, math.inf, math.nan):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    VelocityCommand(**{name: value})
        command = VelocityCommand()
        command.accept(3.0, 0, 0)
        self.assertEqual(command.target(0), (20, 20))


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.driver = LowTempDriver(serial_factory=FakeSerial)
        self.port = self.driver._serial

    def tearDown(self):
        self.port.write_failure = self.port.short_write = False
        with patch("low_temp_driver.time.sleep"):
            self.driver.close()

    def test_serial_defaults_exclusivity_and_feedback_optional(self):
        self.assertEqual(self.port.args, ("/dev/ttyUSB0", 500000))
        self.assertTrue(self.port.kwargs["exclusive"])
        self.assertEqual(self.port.kwargs["timeout"], 0)
        self.driver.set_wheel_speeds(1.5, -2.25)
        self.assertEqual(self.port.writes, [SPEED])
        self.assertEqual(self.driver.read_states(), [])

    def test_receives_state_and_counts_bytes(self):
        self.port.rx.extend(STATE * 3)
        self.assertEqual(self.driver.read_states(), [STATE_VALUES] * 3)
        self.assertEqual(self.driver.rx_bytes, 201)
        self.assertEqual(self.driver.parser.motor_packets, 3)

    def test_invalid_targets_are_never_written(self):
        for left, right in ((math.nan, 0), (21, 0), (0, -21)):
            with self.assertRaises(ValueError):
                self.driver.set_wheel_speeds(left, right)
        self.assertEqual(self.port.writes, [])

    def test_short_write_is_a_failure_not_success(self):
        self.port.short_write = True
        with self.assertRaises(OSError):
            self.driver.set_wheel_speeds(1, 1)
        self.assertEqual(self.driver.tx_packets, 0)

    def test_close_repeats_zero_and_closes_port(self):
        with patch("low_temp_driver.time.sleep"):
            self.driver.close()
        self.assertEqual(self.port.writes, [ZERO] * 5)
        self.assertFalse(self.port.is_open)

    def test_close_still_closes_after_write_failure(self):
        self.port.write_failure = True
        with self.assertRaises(OSError):
            self.driver.close()
        self.assertFalse(self.port.is_open)


# Minimal ROS doubles: exercising the real node callbacks and core together.
class Message:
    def __init__(self, data=None):
        self.data = data
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class Timer:
    def __init__(self, period, callback, clock):
        self.period, self.callback, self.clock = period, callback, clock
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.name, self.params, self.descriptors, self.timers = name, {}, {}, []
        self.logs = []

    def declare_parameter(self, name, value, descriptor):
        self.params[name] = self.overrides.get(name, value)
        self.descriptors[name] = descriptor

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.params[name])

    def create_publisher(self, *args):
        messages = []
        return types.SimpleNamespace(publish=messages.append, messages=messages)

    def create_subscription(self, kind, topic, callback, qos):
        return types.SimpleNamespace(topic_name="/" + topic, qos=qos, callback=callback)

    def create_timer(self, period, callback, clock):
        timer = Timer(period, callback, clock)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return types.SimpleNamespace(info=self.logs.append, warning=self.logs.append,
                                     error=self.logs.append)

    def destroy_node(self):
        pass


def load_node_with_ros_doubles():
    modules = {}
    specs = {
        "rclpy": {},
        "rclpy.node": {"Node": FakeNode},
        "rclpy.clock": {"Clock": types.SimpleNamespace,
                        "ClockType": types.SimpleNamespace(STEADY_TIME="steady")},
        "rclpy.executors": {"ExternalShutdownException": type("Shutdown", (Exception,), {})},
        "rclpy.qos": {"QoSProfile": types.SimpleNamespace,
                      "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT="best_effort"),
                      "DurabilityPolicy": types.SimpleNamespace(VOLATILE="volatile")},
        "geometry_msgs": {}, "geometry_msgs.msg": {"Twist": Message},
        "std_msgs": {}, "std_msgs.msg": {"Float64MultiArray": Message},
        "rcl_interfaces": {},
        "rcl_interfaces.msg": {"ParameterDescriptor": types.SimpleNamespace},
    }
    for name, attributes in specs.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    spec = importlib.util.spec_from_file_location(
        "node_under_test", Path(__file__).resolve().parents[1] / "low_temp_driver_node.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


NODE = load_node_with_ros_doubles()


class NodeCallbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.node = NODE.LowTempDriverNode(serial_factory=FakeSerial, monotonic=lambda: self.now)
        self.port = self.node._base._serial

    def tearDown(self):
        self.port.write_failure = self.port.read_failure = self.port.short_write = False
        with patch("low_temp_driver.time.sleep"):
            self.node.close()
        FakeNode.overrides = {}

    def send(self, linear=0.0, angular=0.0):
        msg = Message()
        msg.linear.x, msg.angular.z = linear, angular
        self.node._on_cmd_vel(msg)

    def test_zero_until_command_then_full_target_without_feedback(self):
        self.node._update()
        self.assertEqual(self.port.writes[-1], ZERO)
        self.send(1.5)
        self.node._update()
        self.assertEqual(self.port.writes[-1], encode_doubles(MOTOR_SPEED_ID, (10, 10)))
        self.assertEqual(self.node._command_pub.messages[-1].data, [10, 10])
        self.assertEqual(self.node._state_pub.messages, [])

    def test_periodic_resend_then_timeout_then_recovery(self):
        self.send(0.3)
        expected = encode_doubles(MOTOR_SPEED_ID, (2, 2))
        for offset in (0, 0.05, 0.10, 0.45):
            self.now = 10.0 + offset
            self.node._update()
            self.assertEqual(self.port.writes[-1], expected)
        self.now = 10.5
        self.node._update()
        self.assertEqual(self.port.writes[-1], ZERO)
        self.send(0.3)
        self.node._update()
        self.assertEqual(self.port.writes[-1], expected)

    def test_feedback_is_published_in_vendor_order(self):
        self.port.rx.extend(STATE * 2)
        self.node._update()
        self.assertEqual([m.data for m in self.node._state_pub.messages],
                         [list(STATE_VALUES)] * 2)

    def test_invalid_new_command_sends_zero_on_next_tick(self):
        self.send(0.3)
        self.node._update()
        self.send(3.1)
        self.node._update()
        self.assertEqual(self.port.writes[-1], ZERO)
        self.assertEqual(self.node._cmd_rejected, 1)

    def test_disconnect_raises_fatal_error_and_cancels_timer(self):
        self.port.write_failure = True
        with self.assertRaises(NODE.SerialLinkLost):
            self.node._update()
        self.assertTrue(self.node._timer.cancelled)

    def test_read_failure_attempts_zero_before_exiting(self):
        self.port.read_failure = True
        self.send(0.3)
        with patch("low_temp_driver.time.sleep"), self.assertRaises(NODE.SerialLinkLost):
            self.node._update()
        self.assertEqual(self.port.writes[-5:], [ZERO] * 5)

    def test_failed_write_does_not_publish_as_sent_target(self):
        self.port.short_write = True
        self.send(0.3)
        with self.assertRaises(NODE.SerialLinkLost):
            self.node._update()
        self.assertEqual(self.node._command_pub.messages, [])

    def test_clock_qos_and_parameters(self):
        self.assertEqual(self.node._timer.period, 0.05)
        self.assertEqual(self.node._timer.clock.clock_type, "steady")
        self.assertEqual(self.node._subscription.qos.depth, 1)
        self.assertEqual(self.node._subscription.qos.reliability, "best_effort")
        self.assertTrue(all(d.read_only for d in self.node.descriptors.values()))

    def test_status_distinguishes_no_command_no_feedback_and_sent_target(self):
        self.node._log_status()
        self.assertIn("reason=waiting_cmd_vel", self.node.logs[-1])
        self.assertIn("feedback_age=never", self.node.logs[-1])
        self.send(0.3)
        self.node._update()
        self.node._log_status()
        self.assertIn("TX_wheels=(2.000,2.000)", self.node.logs[-1])
        self.assertIn("cmd_rx=1", self.node.logs[-1])
        self.assertIn("motor_packets=0", self.node.logs[-1])

    def test_invalid_timer_configuration_never_opens_serial(self):
        for name, value in (("control_rate", 4.0), ("status_period", -1.0)):
            opened = []
            FakeNode.overrides = {name: value}
            with self.assertRaises(ValueError):
                NODE.LowTempDriverNode(serial_factory=lambda *a, **k: opened.append(True))
            self.assertEqual(opened, [])


if __name__ == "__main__":
    unittest.main()
