"""Hardware-free regression tests. All serial and ROS calls are fakes.

Run from the package directory: python3 -m unittest discover -s tests -v
"""
import json
import math
from pathlib import Path
import random
import struct
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class FakeSerial:
    instances = []

    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.rx = bytearray()
        self.writes = []
        self.write_error = False
        self.partial_write = False
        self.instances.append(self)

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        value = bytes(self.rx[:size])
        del self.rx[:size]
        return value

    def write(self, data):
        if self.write_error:
            raise OSError("simulated unplug")
        self.writes.append(bytes(data))
        return len(data) - 1 if self.partial_write else len(data)

    def close(self):
        self.is_open = False


serial = types.ModuleType("serial")
serial.Serial = FakeSerial
serial.SerialException = type("SerialException", (OSError,), {})
sys.modules["serial"] = serial


class Message:
    def __init__(self):
        self.data = None


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
        self.angular = types.SimpleNamespace(x=0., y=0., z=0.)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.logs = []
        self.subscriptions = []

    def declare_parameter(self, name, value):
        self.parameters[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        subscription = types.SimpleNamespace(topic_name="/" + args[1].lstrip("/"), qos=args[3])
        self.subscriptions.append(subscription)
        return subscription

    def create_service(self, *args):
        return None

    def create_timer(self, period, callback):
        return types.SimpleNamespace(period=period, callback=callback)

    def get_logger(self):
        return types.SimpleNamespace(info=self.logs.append, warning=self.logs.append, error=self.logs.append)


for name in ("rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
             "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["rclpy.node"].Node = FakeNode
sys.modules["rclpy.qos"].QoSProfile = lambda **kw: kw
sys.modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
sys.modules["rclpy.qos"].DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
sys.modules["geometry_msgs.msg"].Twist = Twist
for name in ("Float64", "Float64MultiArray", "String"):
    setattr(sys.modules["std_msgs.msg"], name, Message)
sys.modules["std_srvs.srv"].Trigger = Message

from mf_mobile_driver import MfMobileDriver as Driver, MotionConfig, CasterMotionProfile
import mf_mobile_driver_node as node_module


def frame(packet_id, values, ack=0, count=None):
    payload = b"".join(bytes((kind, 8 if kind == 2 else 4)) +
                       struct.pack("<d" if kind == 2 else "<i", value)
                       for kind, value in values)
    body = b"\xff\xf9" + bytes((3 + len(payload), packet_id, ack,
                                  len(values) if count is None else count)) + payload
    return body + bytes((Driver._checksum(body),))


def motor(left=0., right=0.):
    return frame(1, [(2, x) for x in (left, 0., 0., right, 0., 0.)])


def lift(position=0., flags=33):
    return frame(5, [(2, position), (2, 100.), (0, flags)])


class TransportTests(unittest.TestCase):
    def test_command_wire_format_unchanged(self):
        # Fixed wire bytes captured from the original driver, independent of this implementation.
        vectors = json.loads((ROOT / "tests/fixtures/command_packets.json").read_text())
        for item in vectors["speed"]:
            self.assertEqual(Driver._speed_packet(item["left"], item["right"]), bytes.fromhex(item["hex"]))
        for item in vectors["lift"]:
            self.assertEqual(Driver._lift_packet(item["command"]), bytes.fromhex(item["hex"]))

    def test_fragmented_packet_all_split_positions(self):
        packet = motor(1., 2.)
        for split in range(1, len(packet)):
            d = Driver()
            d._serial.rx.extend(packet[:split])
            self.assertEqual(d.read_states(), [])
            d._serial.rx.extend(packet[split:])
            self.assertEqual(d.read_states(), [(1., 0., 0., 2., 0., 0.)])

    def test_bad_length_does_not_swallow_following_frame(self):
        d = Driver()
        bad = bytearray(motor())
        bad[2] += 10
        d._serial.rx.extend(bad + motor(1., 2.))
        self.assertEqual(d.read_states(), [(1., 0., 0., 2., 0., 0.)])

    def test_bad_crc_recovers(self):
        d = Driver()
        bad = bytearray(motor())
        bad[-1] ^= 1
        d._serial.rx.extend(b"noise" + bad + motor())
        self.assertEqual(len(d.read_states()), 1)
        self.assertGreater(d.bad_frames, 0)

    def test_lift_count_ack_and_finite_checks(self):
        d = Driver(enable_lift=True)
        for packet in (frame(5, [(2, .1), (2, 100.), (0, 33)], count=0),
                       frame(5, [(2, .1), (2, 100.), (0, 33)], ack=255),
                       lift(float("nan")), lift(float("inf"))):
            d._serial.rx.extend(packet)
            self.assertEqual(d.read_lift_states(), [])
        d._serial.rx.extend(lift(.1))
        self.assertEqual(d.read_lift_states(), [(.1, 100., 33)])

    def test_nonfinite_motor_state_rejected(self):
        d = Driver()
        d._serial.rx.extend(motor(float("nan"), 0.))
        self.assertEqual(d.read_states(), [])

    def test_partial_write_is_an_error(self):
        d = Driver()
        d._serial.partial_write = True
        with self.assertRaises(serial.SerialException):
            d.set_velocity(.1, 0.)
        self.assertEqual(d.tx_packets, 0)
        self.assertEqual(d.tx_bytes, len(Driver._speed_packet(.1/.12, .1/.12)) - 1)

    def test_serial_counters_distinguish_raw_and_valid_feedback(self):
        d = Driver()
        d._serial.rx.extend(b"garbage")
        self.assertEqual(d.read_states(), [])
        self.assertEqual(d.rx_bytes, 7)
        self.assertEqual(d.motor_packets, 0)
        packet = motor(1., 2.)
        d._serial.rx.extend(packet)
        self.assertEqual(len(d.read_states()), 1)
        self.assertEqual(d.rx_bytes, 7 + len(packet))
        self.assertEqual(d.motor_packets, 1)
        self.assertEqual(d.last_rx_hex, packet.hex(" "))
        d.set_velocity(.1, 0.)
        self.assertEqual(d.tx_packets, 1)
        self.assertEqual(d.tx_bytes, len(d._serial.writes[-1]))

    def test_raw_preview_is_bounded(self):
        d = Driver()
        d._serial.rx.extend(b"x" * 1000)
        d.read_states()
        self.assertEqual(d.rx_bytes, 1000)
        self.assertEqual(len(bytes.fromhex(d.last_rx_hex)), 128)

    def test_disabled_lift_never_transmits_lift_stop(self):
        d = Driver(enable_lift=False)
        d.stop(repeat=2, interval=0)
        self.assertEqual([p[3] for p in d._serial.writes], [2, 2])

    def test_rx_backlog_is_not_treated_as_fresh(self):
        d = Driver()
        d._serial.rx.extend(b"x" * 8193)
        with self.assertRaises(serial.SerialException):
            d.read_states()


class MotionTests(unittest.TestCase):
    def test_startup_cap_and_acceleration(self):
        p = CasterMotionProfile(MotionConfig())
        previous = 0.
        for i in range(17):
            left, right = p.step(.3, 0., i * .05, .05, True)
            self.assertLessEqual(abs(left - previous), .100000001)
            self.assertLessEqual(abs(left * .12), .080000001)
            self.assertEqual(left, right)
            previous = left

    def test_arc_ratio_preserved_at_launch(self):
        p = CasterMotionProfile(MotionConfig())
        for i in range(30):
            left, right = p.step(.2, .2, i * .05, .05, True)
            if abs(right) > 1e-9:
                self.assertAlmostEqual(left / right, (.2 - .2 * .35) / (.2 + .2 * .35))

    def test_reverse_requires_measured_stop_and_pause(self):
        p = CasterMotionProfile(MotionConfig())
        for i in range(50):
            p.step(.2, 0., i * .05, .05, True)
        for i in range(50, 100):
            left, right = p.step(-.2, 0., i * .05, .05, False)
            self.assertGreaterEqual(left, 0.)
            self.assertGreaterEqual(right, 0.)
        self.assertEqual(p.phase, "reversal_wait")
        for i in range(100, 104):
            self.assertEqual(p.step(-.2, 0., i * .05, .05, True), (0., 0.))
        self.assertLess(p.step(-.2, 0., 5.3, .05, True)[0], 0.)

    def test_zero_bypasses_ramp_and_restart_waits_for_actual_stop(self):
        p = CasterMotionProfile(MotionConfig())
        for i in range(40):
            p.step(.3, 0., i * .05, .05, True)
        self.assertEqual(p.step(0., 0., 2., .05, False), (0., 0.))
        self.assertEqual(p.step(-.3, 0., 2.05, .05, False), (0., 0.))

    def test_random_commands_respect_wheel_rate_and_speed_limits(self):
        p = CasterMotionProfile(MotionConfig())
        rng = random.Random(761)
        previous = (0., 0.)
        for i in range(2000):
            result = p.step(rng.uniform(-3, 3), rng.uniform(-4, 4), i * .05, .05, True)
            for before, after in zip(previous, result):
                self.assertLessEqual(abs(after), 20.00000001)
                self.assertLessEqual(abs(after - before), .20000001)
            previous = result


class NodeTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        node_module.time = types.SimpleNamespace(monotonic=lambda: self.now)
        FakeNode.overrides = {"require_motor_feedback": True}

    def prime(self, enable_lift=False):
        FakeNode.overrides = {"enable_lift": enable_lift, "lift_max_height": .5 if enable_lift else 0.,
                              "require_motor_feedback": True}
        self.node = node_module.MfMobileDriverNode()
        self.tick(motor_packet=None)
        self.tick(lift_packets=lift() if enable_lift else b"")
        return self.node

    def tick(self, dt=.05, motor_packet=True, lift_packets=b""):
        self.now += dt
        if self.node._base is not None:
            port = self.node._base._serial
            if motor_packet is not None:
                port.rx.extend(motor() if motor_packet is True else motor_packet)
            port.rx.extend(lift_packets)
        self.node._update()

    def command(self, linear=.2, angular=0.):
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        self.node._on_cmd_vel(message)

    def output(self):
        packets = [p for p in self.node._base._serial.writes if p[3] == 2]
        packet = packets[-1]
        return struct.unpack_from("<d", packet, 8)[0], struct.unpack_from("<d", packet, 18)[0]

    def lift_output(self):
        packet = [p for p in self.node._base._serial.writes if p[3] == 6][-1]
        return struct.unpack_from("<i", packet, 8)[0]

    def move(self):
        for _ in range(30):
            self.command()
            self.tick()
        self.assertGreater(self.output()[0], 0.)

    def test_no_feedback_means_no_drive(self):
        self.node = node_module.MfMobileDriverNode()
        self.tick(motor_packet=None)
        for _ in range(20):
            self.command()
            self.tick(motor_packet=None)
        self.assertEqual(self.output(), (0., 0.))

    def optional_node(self):
        FakeNode.overrides = {}
        self.node = node_module.MfMobileDriverNode()
        self.tick(motor_packet=None)

    def move_without_feedback(self, linear=.2, ticks=30):
        for _ in range(ticks):
            self.command(linear)
            self.tick(motor_packet=None)

    def test_default_drives_without_any_feedback(self):
        self.optional_node()
        self.assertFalse(self.node._require_motor_feedback)
        self.move_without_feedback()
        self.assertGreater(self.output()[0], 0.)
        self.assertEqual(self.output()[0], self.output()[1])
        self.assertFalse(self.node._fault)

    def test_ten_hz_commands_without_feedback_reach_requested_speed(self):
        self.optional_node()
        for step in range(60):
            if step % 2 == 0:
                self.command(.1)
            self.tick(motor_packet=None)
        self.assertAlmostEqual(self.output()[0] * .12, .1)
        self.assertEqual(self.node._cmd_received, 30)
        self.assertEqual(self.node._cmd_accepted, 30)
        self.assertEqual(self.node._command_status(self.now), "nonzero_command_sent")

    def test_starting_node_without_cmd_never_requests_motion(self):
        self.optional_node()
        for _ in range(40):
            self.tick(motor_packet=None)
        self.assertEqual(self.output(), (0., 0.))
        self.assertEqual(self.node._command_status(self.now), "waiting_cmd_vel")
        self.assertTrue(any("cmd_rx=0" in log and "reason=waiting_cmd_vel" in log for log in self.node.logs))

    def test_timeout_reason_is_preserved_and_reported_once(self):
        self.optional_node()
        self.move_without_feedback()
        for _ in range(40):
            self.tick(motor_packet=None)
        self.assertEqual(self.node._drive_state, "cmd_vel_timeout")
        self.assertEqual(self.node._command_status(self.now), "cmd_vel_timeout")
        self.assertEqual(sum("CMD_TIMEOUT:" in log for log in self.node.logs), 1)
        self.assertEqual(self.output(), (0., 0.))

    def test_cmd_qos_accepts_both_publisher_reliabilities_by_default(self):
        self.optional_node()
        self.assertEqual(self.node._cmd_sub.qos["reliability"], 2)
        self.assertEqual(self.node._cmd_topic, "/cmd_vel")
        FakeNode.overrides = {"cmd_vel_reliability": "reliable", "cmd_vel_topic": "robot/velocity"}
        node = node_module.MfMobileDriverNode()
        self.assertEqual(node._cmd_sub.qos["reliability"], 1)
        self.assertEqual(node._cmd_topic, "/robot/velocity")

    def test_rejected_command_reason_and_counts_are_visible(self):
        self.optional_node()
        self.node._on_stop(None, Message())
        self.command()
        self.assertEqual(self.node._cmd_rejected, 1)
        self.assertEqual(self.node._last_rejection, "operator_stop")
        self.assertTrue(any("CMD_RX rejected: operator_stop" in s for s in self.node.logs))
        self.node._on_reset(None, Message())
        self.assertEqual(self.node._command_status(self.now), "waiting_new_cmd_vel")

    def test_status_counts_no_rx_and_valid_input_are_distinct(self):
        self.optional_node()
        self.move_without_feedback()
        status = self.node._diagnostics(self.now)
        self.assertGreater(status["serial_tx_packets"], 0)
        self.assertEqual(status["serial_rx_bytes"], 0)
        self.assertEqual(status["motor_state_packets"], 0)
        self.assertEqual(status["cmd_received_count"], 30)
        self.assertGreater(status["transmitted_wheel_speeds"][0], 0.)

    def test_raw_serial_logging_does_not_publish_fake_feedback(self):
        self.optional_node()
        self.node._p["debug_serial"] = True
        self.node._base._serial.rx.extend(b"not a valid motor frame")
        for _ in range(25):
            self.tick(motor_packet=None)
        self.assertTrue(any("RX_HEX last read" in s for s in self.node.logs))
        self.assertEqual(self.node._state_pub.messages, [])
        self.assertGreater(self.node._base.rx_bytes, 0)
        self.assertEqual(self.node._base.motor_packets, 0)

    def test_invalid_diagnostic_options_rejected(self):
        for option in ({"cmd_vel_reliability": "automatic"}, {"debug_serial": "false"}, {"diagnostic_period": 0.}):
            FakeNode.overrides = option
            with self.assertRaises(ValueError):
                node_module.MfMobileDriverNode()

    def test_optional_does_not_publish_fake_feedback(self):
        import json
        self.optional_node()
        self.move_without_feedback()
        self.assertEqual(self.node._state_pub.messages, [])
        status = json.loads(self.node._status_pub.messages[-1].data)
        self.assertFalse(status["motor_feedback_required"])
        self.assertFalse(status["motor_feedback_fresh"])
        self.assertFalse(status["standstill_verified"])
        self.assertGreater(self.node._applied_pub.messages[-1].linear.x, 0.)

    def test_optional_feedback_loss_does_not_latch_or_stop(self):
        self.optional_node()
        self.tick()
        self.move_without_feedback(ticks=40)
        self.assertFalse(self.node._fault)
        self.assertGreater(self.output()[0], 0.)

    def test_optional_command_timeout_still_stops(self):
        self.optional_node()
        self.move_without_feedback()
        for _ in range(11):
            self.tick(motor_packet=None)
        self.assertEqual(self.output(), (0., 0.))

    def test_optional_reverse_has_zero_dwell(self):
        self.optional_node()
        self.move_without_feedback()
        first_zero = None
        first_negative = None
        for _ in range(60):
            self.command(-.2)
            self.tick(motor_packet=None)
            value = self.output()[0]
            if value == 0. and first_zero is None:
                first_zero = self.now
            if value < 0.:
                first_negative = self.now
                break
        self.assertIsNotNone(first_zero)
        self.assertIsNotNone(first_negative)
        self.assertGreaterEqual(first_negative - first_zero, .20)

    def test_optional_stop_reset_does_not_replay_old_command(self):
        self.optional_node()
        self.move_without_feedback()
        self.assertTrue(self.node._on_stop(None, Message()).success)
        self.move_without_feedback()
        self.assertEqual(self.output(), (0., 0.))
        response = self.node._on_reset(None, Message())
        self.assertTrue(response.success)
        self.assertIn("UNVERIFIED", response.message)
        self.tick(motor_packet=None)
        self.assertEqual(self.output(), (0., 0.))
        self.move_without_feedback()
        self.assertGreater(self.output()[0], 0.)

    def test_optional_fresh_moving_feedback_still_blocks_departure_and_reset(self):
        self.optional_node()
        for _ in range(20):
            self.command()
            self.tick(motor_packet=motor(.5, .5))
        self.assertEqual(self.output(), (0., 0.))
        self.node._on_stop(None, Message())
        self.assertFalse(self.node._on_reset(None, Message()).success)

    def test_optional_serial_error_still_latches(self):
        self.optional_node()
        self.move_without_feedback()
        self.node._base._serial.write_error = True
        self.tick(motor_packet=None)
        self.assertIsNone(self.node._base)
        self.assertTrue(self.node._fault.startswith("serial_error"))

    def test_optional_loop_delay_still_latches(self):
        self.optional_node()
        self.move_without_feedback()
        self.tick(dt=.30, motor_packet=None)
        self.assertEqual(self.node._fault, "control_loop_delayed")

    def test_feedback_option_rejects_nonboolean(self):
        FakeNode.overrides = {"require_motor_feedback": "false"}
        with self.assertRaises(ValueError):
            node_module.MfMobileDriverNode()

    def test_command_timeout_stops_without_ramp(self):
        self.prime()
        self.move()
        for _ in range(11):
            self.tick()
        self.assertEqual(self.output(), (0., 0.))

    def test_zero_and_invalid_command_stop(self):
        self.prime()
        self.move()
        self.command(0.)
        self.tick()
        self.assertEqual(self.output(), (0., 0.))
        self.move()
        self.command(float("nan"))
        self.tick()
        self.assertEqual(self.output(), (0., 0.))

    def test_motor_timeout_latched_even_on_feedback_return(self):
        self.prime()
        self.move()
        for _ in range(12):
            self.command()
            self.tick(motor_packet=None)
        self.assertEqual(self.node._fault, "motor_feedback_timeout")
        self.assertEqual(self.output(), (0., 0.))
        self.tick()
        self.command()
        self.tick()
        self.assertEqual(self.output(), (0., 0.))
        response = self.node._on_reset(None, Message())
        self.assertTrue(response.success)
        self.tick()
        self.assertEqual(self.output(), (0., 0.))
        self.move()

    def test_loop_stall_latches_stop(self):
        self.prime()
        self.move()
        self.tick(dt=.30)
        self.assertEqual(self.node._fault, "control_loop_delayed")
        self.assertEqual(self.output(), (0., 0.))

    def test_reset_rejects_moving_wheels(self):
        self.prime()
        self.node._on_stop(None, Message())
        self.tick(motor_packet=motor(.5, .5))
        self.assertFalse(self.node._on_reset(None, Message()).success)

    def test_serial_reconnect_does_not_resume_old_command(self):
        self.prime()
        self.move()
        self.node._base._serial.write_error = True
        self.tick()
        self.assertIsNone(self.node._base)
        for _ in range(25):
            self.tick()
        self.assertIsNotNone(self.node._base)
        self.assertTrue(self.node._fault)
        self.assertEqual(self.output(), (0., 0.))

    def test_lift_requires_explicit_height_limit(self):
        FakeNode.overrides = {"enable_lift": True}
        with self.assertRaises(ValueError):
            node_module.MfMobileDriverNode()

    def test_lift_stale_feedback_clears_target_and_stops(self):
        self.prime(True)
        self.node._on_lift_target(types.SimpleNamespace(data=.3))
        self.tick(lift_packets=lift(.1))
        self.assertEqual(self.lift_output(), Driver.LIFT_UP)
        for _ in range(12):
            self.tick()
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)
        self.assertIsNone(self.node._lift_target)
        self.assertTrue(self.node._lift_fault)
        self.tick(lift_packets=lift(.1))
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)

    def test_lift_sensor_fault_stops(self):
        self.prime(True)
        self.node._on_lift_target(types.SimpleNamespace(data=.3))
        self.tick(lift_packets=lift(.1, 32 | 8))
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)
        self.assertTrue(self.node._lift_fault)

    def test_home_transitions_in_same_batch_complete(self):
        self.prime(True)
        self.tick(lift_packets=lift(.2, 33))
        response = self.node._on_lift_home(None, Message())
        self.assertTrue(response.success)
        self.tick(lift_packets=lift(.2, 33))
        self.assertEqual(self.lift_output(), Driver.LIFT_HOME)
        self.tick(lift_packets=lift(.1, 1 | 16) + lift(0., 1 | 32))
        self.assertFalse(self.node._homing)
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)
        self.assertEqual(self.node._lift_state, "homed")

    def test_home_no_start_times_out(self):
        self.prime(True)
        self.tick(lift_packets=lift(.2, 33))
        self.node._on_lift_home(None, Message())
        for _ in range(45):
            self.tick(lift_packets=lift(.2, 33))
        self.assertEqual(self.node._lift_fault, "home_start_not_observed")
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)

    def test_home_service_reports_fault(self):
        self.prime(True)
        self.node._on_stop(None, Message())
        self.assertFalse(self.node._on_lift_home(None, Message()).success)

    def test_goal_limit_and_completion(self):
        self.prime(True)
        self.node._on_lift_target(types.SimpleNamespace(data=.6))
        self.assertEqual(self.node._lift_fault, "target_out_of_range")
        self.assertTrue(self.node._on_reset(None, Message()).success)
        self.node._on_lift_target(types.SimpleNamespace(data=.3))
        self.tick(lift_packets=lift(.29))
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)
        self.assertIsNone(self.node._lift_target)
        self.assertEqual(self.node._lift_state, "target_reached")

    def test_lift_repeated_goal_cannot_extend_deadline(self):
        self.prime(True)
        self.node._p["lift_motion_timeout"] = .3
        for _ in range(10):
            self.node._on_lift_target(types.SimpleNamespace(data=.3))
            self.tick(lift_packets=lift(.1))
        self.assertEqual(self.node._lift_fault, "motion_timeout")
        self.assertEqual(self.lift_output(), Driver.LIFT_STOP)


if __name__ == "__main__":
    unittest.main()
