#!/usr/bin/env python3
"""ROS 2 adapter: passive-caster motion shaping and explicit fault recovery."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger
import serial

from mf_mobile_driver import MfMobileDriver, MotionConfig, CasterMotionProfile


SENSOR_VALID, WIRE_FAULT, HOMING, HOMED = 1, 8, 16, 32
IO_ERRORS = (OSError, RuntimeError, ValueError, serial.SerialException)


class MfMobileDriverNode(Node):
    def __init__(self):
        super().__init__("mf_mobile_driver")
        defaults = dict(
            port="/dev/ttyUSB0", baudrate=921600, enable_lift=False,
            cmd_vel_topic="cmd_vel", motor_state_topic="motor_state",
            lift_state_topic="lift_state", lift_target_topic="lift_target",
            control_rate=20.0, cmd_vel_timeout=0.5,
            motor_state_timeout=0.5, lift_state_timeout=0.5,
            require_motor_feedback=False,
            cmd_vel_reliability="best_effort", diagnostic_period=1.0, debug_serial=False,
            control_loop_timeout=0.25, reconnect_interval=1.0,
            stopped_wheel_speed=0.10,
            lift_tolerance=0.02, lift_max_height=0.0,
            lift_motion_timeout=30.0, lift_home_timeout=30.0,
            lift_home_start_timeout=2.0,
        )
        defaults.update(vars(MotionConfig()))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = {name: self.get_parameter(name).value for name in defaults}
        if not isinstance(self._p["enable_lift"], bool):
            raise ValueError("enable_lift must be boolean")
        self._enable_lift = self._p["enable_lift"]
        if not isinstance(self._p["require_motor_feedback"], bool):
            raise ValueError("require_motor_feedback must be boolean")
        self._require_motor_feedback = self._p["require_motor_feedback"]
        if self._p["cmd_vel_reliability"] not in ("best_effort", "reliable"):
            raise ValueError("cmd_vel_reliability must be best_effort or reliable")
        if not isinstance(self._p["debug_serial"], bool):
            raise ValueError("debug_serial must be boolean")
        numeric = ("control_rate", "cmd_vel_timeout", "motor_state_timeout",
                   "lift_state_timeout", "control_loop_timeout", "reconnect_interval",
                   "stopped_wheel_speed", "lift_tolerance", "lift_motion_timeout",
                   "lift_home_timeout", "lift_home_start_timeout", "diagnostic_period")
        for name in numeric:
            value = float(self._p[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and > 0")
            self._p[name] = value
        if self._p["control_loop_timeout"] < 2.0 / self._p["control_rate"]:
            raise ValueError("control_loop_timeout must be >= two control periods")
        lift_max = float(self._p["lift_max_height"])
        if not math.isfinite(lift_max) or lift_max < 0 or (self._enable_lift and lift_max <= 0):
            raise ValueError("set lift_max_height to the measured allowed height when enabling lift")
        self._p["lift_max_height"] = lift_max
        motion = MotionConfig(**{name: float(self._p[name]) for name in vars(MotionConfig())})
        self._motion = CasterMotionProfile(motion)
        self._base_args = {name: self._p[name] for name in (
            "port", "baudrate", "enable_lift", "wheel_radius", "wheel_separation", "max_wheel_speed")}
        # Validate before opening the hardware, including transport-only settings.
        MfMobileDriver._require_positive("baudrate", self._p["baudrate"])
        if int(self._p["baudrate"]) != self._p["baudrate"]:
            raise ValueError("baudrate must be an integer")
        self._base = None
        self._next_connect = 0.0
        self._last_motor = self._last_lift = self._last_command = None
        self._motor = (0.0,) * 6
        self._linear = self._angular = 0.0
        self._fault = ""
        self._drive_state = "waiting_feedback"
        self._lift_fault = ""
        self._lift_state = "disabled" if not self._enable_lift else "waiting_feedback"
        self._lift_position = self._lift_flags = 0
        self._lift_target = None
        self._lift_started_at = None
        self._homing = self._home_seen = self._home_sent = False
        self._home_initial_homed = False
        self._home_started_at = None
        self._last_update = time.monotonic()
        self._last_diagnostic_at = self._last_update
        self._cmd_received = self._cmd_accepted = self._cmd_rejected = 0
        self._last_received_at = None
        self._last_rejection = ""
        self._last_sent_wheels = (0.0, 0.0)
        self._diagnostic_rx_bytes = 0
        self._last_status_at = None
        self._last_status = None
        self._state_pub = self.create_publisher(Float64MultiArray, self._p["motor_state_topic"], 10)
        self._status_pub = self.create_publisher(String, "driver_status", 10)
        self._applied_pub = self.create_publisher(Twist, "cmd_vel_applied", 10)
        # Keep only the newest velocity, with no retained command after restart.
        # BEST_EFFORT subscriptions also accept RELIABLE publishers. Timeout
        # remains active; no retained command is replayed after restart.
        reliability = (ReliabilityPolicy.BEST_EFFORT if self._p["cmd_vel_reliability"] == "best_effort"
                       else ReliabilityPolicy.RELIABLE)
        qos = QoSProfile(depth=1, reliability=reliability,
                         durability=DurabilityPolicy.VOLATILE)
        self._cmd_sub = self.create_subscription(Twist, self._p["cmd_vel_topic"], self._on_cmd_vel, qos)
        self._cmd_topic = self._cmd_sub.topic_name
        self.create_service(Trigger, "driver_stop", self._on_stop)
        self.create_service(Trigger, "driver_reset", self._on_reset)
        if self._enable_lift:
            self._lift_state_pub = self.create_publisher(Float64MultiArray, self._p["lift_state_topic"], 10)
            self.create_subscription(Float64, self._p["lift_target_topic"], self._on_lift_target, 1)
            self.create_service(Trigger, "lift_set_home", self._on_lift_home)
            self.create_service(Trigger, "lift_stop", self._on_lift_stop)
        self._timer = self.create_timer(1.0 / self._p["control_rate"], self._update)
        self.get_logger().info(
            f"cmd_vel input: {self._cmd_topic} [geometry_msgs/msg/Twist, "
            f"{self._p['cmd_vel_reliability']}]; publish continuously at 10 Hz or faster. "
            "Starting this node alone does not request motion.")
        if self._require_motor_feedback:
            self.get_logger().info("Caster driver ready: fresh motor feedback is required")
        else:
            self.get_logger().warning(
                "Motor feedback is OPTIONAL: cmd_vel can drive without feedback; "
                "actual speed/standstill cannot be verified when feedback is absent. "
                "cmd_vel timeout and serial fault handling remain enabled.")

    def _fresh(self, timestamp, timeout, now):
        return timestamp is not None and 0 <= now - timestamp <= timeout

    def _motor_fresh(self, now):
        return self._fresh(self._last_motor, self._p["motor_state_timeout"], now)

    def _lift_fresh(self, now):
        return self._fresh(self._last_lift, self._p["lift_state_timeout"], now)

    def _measured_stopped(self):
        return max(abs(self._motor[0]), abs(self._motor[3])) <= self._p["stopped_wheel_speed"]

    def _clear_drive(self):
        self._linear = self._angular = 0.0
        self._last_command = None
        self._motion.reset()

    def _clear_lift(self):
        self._lift_target = None
        self._lift_started_at = None
        self._homing = self._home_seen = self._home_sent = False

    def _latch_fault(self, reason):
        if not self._fault:
            self._fault = reason
            self.get_logger().error(f"Driver fault: {reason}; call driver_reset after recovery")
        self._clear_drive()
        self._clear_lift()
        if self._enable_lift:
            self._lift_state = "stopped_by_driver"

    def _fail_lift(self, reason):
        if not self._lift_fault:
            self._lift_fault = reason
            self.get_logger().error(f"Lift fault: {reason}; call driver_reset after recovery")
        self._clear_lift()
        self._lift_state = "fault"

    def _disconnect(self, error, now):
        self._latch_fault(f"serial_error: {error}")
        base, self._base = self._base, None
        self._last_sent_wheels = None  # Failed/disconnected transport has unknown output.
        self._last_motor = self._last_lift = None
        if base is not None:
            try:
                base.close()
            except IO_ERRORS:
                pass
        self._next_connect = now + self._p["reconnect_interval"]

    def _connect(self, now):
        if self._base is not None or now < self._next_connect:
            return
        try:
            self._base = MfMobileDriver(**self._base_args)
            self._base.stop(repeat=1, interval=0)
            self._last_sent_wheels = (0.0, 0.0)
            self._diagnostic_rx_bytes = 0
            self._clear_drive()
            self._clear_lift()
            self.get_logger().info(
                f"Serial connected: port={self._p['port']} baudrate={self._p['baudrate']}; "
                "send a NEW cmd_vel; motor feedback "
                + ("required" if self._require_motor_feedback else "optional"))
        except IO_ERRORS as error:
            self._disconnect(error, now)

    def _on_cmd_vel(self, message):
        now = time.monotonic()
        self._cmd_received += 1
        previous_received = self._last_received_at
        self._last_received_at = now
        linear, angular = message.linear.x, message.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._reject_command("invalid_cmd_vel")
            self._clear_drive()
            self._drive_state = "invalid_cmd_vel"
            self.get_logger().warning("Non-finite cmd_vel rejected; previous command cleared")
            return
        # Reject numerical overflow before the cap/ramp calculations.
        half_turn = angular * self._motion.config.wheel_separation / 2
        raw = ((linear - half_turn) / self._motion.config.wheel_radius,
               (linear + half_turn) / self._motion.config.wheel_radius)
        if not all(map(math.isfinite, raw)):
            self._reject_command("invalid_cmd_vel")
            self._clear_drive()
            self._drive_state = "invalid_cmd_vel"
            return
        if (self._fault or self._base is None
                or (self._require_motor_feedback and not self._motor_fresh(now))):
            self._reject_command(self._fault or ("serial_disconnected" if self._base is None
                                                 else "waiting_motor_feedback"))
            self._clear_drive()
            return
        self._linear, self._angular = linear, angular
        self._last_command = now
        self._cmd_accepted += 1
        self._last_rejection = ""
        if previous_received is None or now - previous_received > self._p["cmd_vel_timeout"]:
            self.get_logger().info(f"CMD_RX accepted: v={linear:.3f} m/s w={angular:.3f} rad/s")

    def _reject_command(self, reason):
        self._cmd_rejected += 1
        if reason != self._last_rejection:
            self.get_logger().warning(f"CMD_RX rejected: {reason}")
        self._last_rejection = reason

    def _poll_feedback(self, now):
        for state in self._base.read_states():
            self._motor = state
            self._last_motor = now
            message = Float64MultiArray()
            message.data = list(state)
            self._state_pub.publish(message)
        if not self._enable_lift:
            return
        for position, adc, flags in self._base.read_lift_states():
            self._lift_position, self._lift_flags = position, flags
            self._last_lift = now
            message = Float64MultiArray()
            message.data = [position, adc, float(flags)]
            self._lift_state_pub.publish(message)
            # Process EVERY state transition, not just the last packet in a batch.
            if self._homing and self._home_sent:
                if flags & WIRE_FAULT or not flags & SENSOR_VALID:
                    self._fail_lift("sensor_fault_during_home")
                else:
                    self._home_seen = self._home_seen or bool(flags & HOMING)
                    if (flags & HOMED and not flags & HOMING
                            and (self._home_seen or not self._home_initial_homed)):
                        self._homing = False
                        self._lift_state = "homed"

    def _send_drive(self, now, dt):
        if self._fault:
            self._drive_state = "fault"
            self._clear_drive()
            left = right = 0.0
        elif self._require_motor_feedback and not self._motor_fresh(now):
            self._drive_state = "waiting_feedback"
            self._clear_drive()
            left = right = 0.0
        elif not self._fresh(self._last_command, self._p["cmd_vel_timeout"], now):
            self._drive_state = "cmd_vel_timeout" if self._last_command is not None or self._drive_state == "cmd_vel_timeout" else "ready"
            if self._last_command is not None:
                self.get_logger().warning(
                    f"CMD_TIMEOUT: no accepted cmd_vel for {self._p['cmd_vel_timeout']:.2f}s; sending zero. "
                    "Use ros2 topic pub --rate 10 (not --once or the default 1 Hz).")
            self._clear_drive()
            left = right = 0.0
        else:
            measured_stopped = self._measured_stopped() if self._motor_fresh(now) else None
            left, right = self._motion.step(
                self._linear, self._angular, now, dt, measured_stopped,
                require_feedback=self._require_motor_feedback)
            self._drive_state = self._motion.phase
        self._base.set_wheel_speeds(left, right)
        self._last_sent_wheels = (left, right)
        c = self._motion.config
        applied = Twist()
        applied.linear.x = (left + right) * c.wheel_radius / 2
        applied.angular.z = (right - left) * c.wheel_radius / c.wheel_separation
        self._applied_pub.publish(applied)

    def _lift_ready_reason(self, now, require_homed=False):
        if self._fault or self._base is None:
            return "driver_not_ready"
        if self._lift_fault:
            return self._lift_fault
        if not self._lift_fresh(now):
            return "lift_feedback_missing"
        if self._lift_flags & WIRE_FAULT or not self._lift_flags & SENSOR_VALID:
            return "lift_sensor_invalid"
        if require_homed and (not self._lift_flags & HOMED or self._lift_flags & HOMING):
            return "lift_not_homed"
        return ""

    def _on_lift_target(self, message):
        now = time.monotonic()
        if (not math.isfinite(message.data) or message.data < 0
                or message.data > self._p["lift_max_height"]):
            self._fail_lift("target_out_of_range")
            return
        reason = self._lift_ready_reason(now, require_homed=True)
        if reason or self._homing:
            self.get_logger().warning(f"Lift target rejected: {reason or 'home_in_progress'}")
            return
        # Repeated same/changed goals cannot extend an already active motion deadline.
        if self._lift_target is None:
            self._lift_started_at = now
        self._lift_target = message.data
        self._lift_state = "moving"

    def _on_lift_home(self, request, response):
        del request
        now = time.monotonic()
        reason = self._lift_ready_reason(now)
        if reason or self._homing:
            response.success = False
            response.message = reason or "home_in_progress"
            return response
        self._clear_lift()
        if (self._lift_flags & HOMED and not self._lift_flags & HOMING
                and abs(self._lift_position) <= self._p["lift_tolerance"]):
            response.success = True
            response.message = "Already homed at the bottom; no HOME motion requested"
            return response
        self._homing = True
        self._home_initial_homed = bool(self._lift_flags & HOMED)
        self._home_started_at = now
        self._lift_state = "homing"
        response.success = True
        response.message = "HOME request accepted; completion is reported by lift_state/driver_status"
        return response

    def _update_lift(self, now):
        if not self._enable_lift:
            return
        active = self._homing or self._lift_target is not None
        reason = self._lift_ready_reason(now)
        if active and reason:
            self._fail_lift(reason)
        command = MfMobileDriver.LIFT_STOP
        if not reason and not self._lift_fault:
            if self._homing:
                elapsed = now - self._home_started_at
                if elapsed > self._p["lift_home_timeout"]:
                    self._fail_lift("home_timeout")
                elif not self._home_seen and elapsed > self._p["lift_home_start_timeout"]:
                    self._fail_lift("home_start_not_observed")
                else:
                    command = MfMobileDriver.LIFT_HOME
            elif self._lift_target is not None:
                if not self._lift_flags & HOMED or self._lift_flags & HOMING:
                    self._fail_lift("home_state_lost")
                elif now - self._lift_started_at > self._p["lift_motion_timeout"]:
                    self._fail_lift("motion_timeout")
                else:
                    error = self._lift_target - self._lift_position
                    if abs(error) <= self._p["lift_tolerance"]:
                        self._clear_lift()
                        self._lift_state = "target_reached"
                    else:
                        command = MfMobileDriver.LIFT_UP if error > 0 else MfMobileDriver.LIFT_DOWN
        self._base.set_lift_command(command)
        if command == MfMobileDriver.LIFT_HOME:
            self._home_sent = True

    def _on_stop(self, request, response):
        del request
        self._latch_fault("operator_stop")
        try:
            if self._base is not None:
                self._base.stop(repeat=1, interval=0)
                self._last_sent_wheels = (0.0, 0.0)
            response.success = self._base is not None
            response.message = "Stop latched; physical standstill is not acknowledged. Reset required."
        except IO_ERRORS as error:
            self._disconnect(error, time.monotonic())
            response.success = False
            response.message = "Stop latched, but serial transmission failed"
        return response

    def _on_lift_stop(self, request, response):
        del request
        self._clear_lift()
        self._lift_state = "stopped"
        try:
            if self._base is None:
                raise RuntimeError("serial disconnected")
            self._base.set_lift_command(MfMobileDriver.LIFT_STOP)
            response.success = True
            response.message = "Lift STOP sent; previous target cleared"
        except IO_ERRORS as error:
            self._disconnect(error, time.monotonic())
            response.success = False
            response.message = str(error)
        return response

    def _on_reset(self, request, response):
        del request
        now = time.monotonic()
        reason = ""
        if self._base is None:
            reason = "serial_connection_required"
        elif self._require_motor_feedback and not self._motor_fresh(now):
            reason = "fresh_motor_feedback_required"
        elif self._motor_fresh(now) and not self._measured_stopped():
            reason = "wheel_standstill_required"
        elif self._enable_lift and (not self._lift_fresh(now)
                or self._lift_flags & WIRE_FAULT or not self._lift_flags & SENSOR_VALID
                or self._lift_flags & (2 | 4 | HOMING)):
            reason = "fresh_valid_stopped_lift_feedback_required"
        response.success = not bool(reason)
        response.message = reason or "Reset complete; send a NEW cmd_vel or lift request"
        if not reason and not self._motor_fresh(now):
            response.message = "Reset complete without motor feedback; standstill UNVERIFIED; send a NEW cmd_vel"
        if not reason:
            self._fault = self._lift_fault = ""
            self._last_rejection = ""
            self._clear_drive()
            self._clear_lift()
            self._drive_state = "ready"
            self._lift_state = "ready" if self._enable_lift else "disabled"
        return response

    def _command_status(self, now):
        if self._fault:
            return "fault:" + self._fault
        if self._base is None:
            return "serial_disconnected"
        if self._last_received_at is None:
            return "waiting_cmd_vel"
        if self._last_rejection:
            return "rejected:" + self._last_rejection
        if self._drive_state == "cmd_vel_timeout":
            return "cmd_vel_timeout"
        if self._last_command is None or not self._fresh(self._last_command, self._p["cmd_vel_timeout"], now):
            return "waiting_new_cmd_vel"
        if abs(self._linear) < 1e-9 and abs(self._angular) < 1e-9:
            return "zero_command"
        if self._motion.phase in ("idle", "reversal_wait"):
            return "waiting_wheel_stop" if self._motor_fresh(now) and not self._measured_stopped() else "zero_dwell"
        return "nonzero_command_sent" if self._last_sent_wheels and any(self._last_sent_wheels) else self._motion.phase

    def _diagnostics(self, now):
        base = self._base
        return dict(cmd_vel_topic=self._cmd_topic, cmd_vel_reliability=self._p["cmd_vel_reliability"],
                    command_status=self._command_status(now), cmd_received_count=self._cmd_received,
                    cmd_accepted_count=self._cmd_accepted, cmd_rejected_count=self._cmd_rejected,
                    last_cmd_rejection=self._last_rejection,
                    cmd_received_age_s=None if self._last_received_at is None else max(0.0, now - self._last_received_at),
                    requested_velocity=[self._linear, self._angular], transmitted_wheel_speeds=self._last_sent_wheels,
                    serial_rx_bytes=base.rx_bytes if base else 0, serial_tx_bytes=base.tx_bytes if base else 0,
                    serial_tx_packets=base.tx_packets if base else 0,
                    motor_state_packets=base.motor_packets if base else 0,
                    lift_state_packets=base.lift_packets if base else 0,
                    bad_frames=base.bad_frames if base else 0)

    def _log_diagnostics(self, now):
        if now - self._last_diagnostic_at < self._p["diagnostic_period"]:
            return
        self._last_diagnostic_at = now
        d = self._diagnostics(now)
        age = "never" if d["cmd_received_age_s"] is None else f"{d['cmd_received_age_s']:.2f}s"
        wheels = "unknown" if self._last_sent_wheels is None else ",".join(f"{v:.3f}" for v in self._last_sent_wheels)
        self.get_logger().info(
            f"IO cmd_rx={self._cmd_received} accepted={self._cmd_accepted} rejected={self._cmd_rejected} "
            f"cmd_age={age} TX_wheels=({wheels})rad/s tx_packets={d['serial_tx_packets']} "
            f"rx_bytes={d['serial_rx_bytes']} motor_packets={d['motor_state_packets']} "
            f"bad_frames={d['bad_frames']} reason={d['command_status']}")
        if self._p["debug_serial"] and self._base is not None:
            self.get_logger().info(f"TX_HEX last completed packet: {self._base.last_tx_hex}")
            if self._base.rx_bytes != self._diagnostic_rx_bytes:
                self.get_logger().info(f"RX_HEX last read, first 128 bytes: {self._base.last_rx_hex}")
            else:
                self.get_logger().info("RX_HEX: no new bytes since previous report")
        self._diagnostic_rx_bytes = d["serial_rx_bytes"]

    def _publish_status(self, now):
        status = dict(state="fault" if self._fault else self._drive_state,
                                 fault=self._fault, phase=self._motion.phase,
                                 serial_connected=self._base is not None,
                                 motor_feedback_fresh=self._motor_fresh(now),
                                 motor_feedback_required=self._require_motor_feedback,
                                 standstill_verified=self._motor_fresh(now) and self._measured_stopped(),
                                 lift_state=self._lift_state, lift_fault=self._lift_fault,
                                 command_status=self._command_status(now))
        status_key = json.dumps(status, sort_keys=True)
        if status_key != self._last_status or self._last_status_at is None or now - self._last_status_at >= 0.5:
            status.update(self._diagnostics(now))
            message = String()
            message.data = json.dumps(status, sort_keys=True, allow_nan=False)
            self._status_pub.publish(message)
            self._last_status, self._last_status_at = status_key, now

    def _update(self):
        now = time.monotonic()
        elapsed = now - self._last_update
        self._last_update = now
        if elapsed > self._p["control_loop_timeout"]:
            self._latch_fault("control_loop_delayed")
        dt = min(max(elapsed, 1e-6), 1.0 / self._p["control_rate"])
        self._connect(now)
        if self._base is not None:
            try:
                # Check age BEFORE polling: recovery cannot hide a previous feedback gap.
                if (self._require_motor_feedback and self._last_motor is not None
                        and not self._motor_fresh(now)):
                    self._latch_fault("motor_feedback_timeout")
                if (self._enable_lift and (self._homing or self._lift_target is not None)
                        and not self._lift_fresh(now)):
                    self._fail_lift("lift_feedback_timeout")
                self._poll_feedback(now)
                self._send_drive(now, dt)
                self._update_lift(now)
            except IO_ERRORS as error:
                self._disconnect(error, now)
        self._publish_status(now)
        self._log_diagnostics(now)

    def close(self):
        if self._base is not None:
            try:
                self._base.close()
            except IO_ERRORS as error:
                self.get_logger().warning(f"Final STOP failed: {error}")
            self._base = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MfMobileDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
