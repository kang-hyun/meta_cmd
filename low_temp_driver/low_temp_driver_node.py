#!/usr/bin/env python3
"""ROS 2 node for the separate low-temperature robot (mobile_keti 2026-11)."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from low_temp_driver import LowTempDriver, VelocityCommand


class SerialLinkLost(RuntimeError):
    """Fatal transport failure. Restart the node to reconnect."""


class LowTempDriverNode(Node):
    def __init__(self, serial_factory=None, monotonic=time.monotonic):
        super().__init__("low_temp_driver")
        self._base = None
        self._now = monotonic
        try:
            self._configure(serial_factory)
        except Exception:
            if self._base is not None:
                try:
                    self._base.close()
                except OSError:
                    pass
            self.destroy_node()
            raise

    def _configure(self, serial_factory):
        defaults = {
            "port": "/dev/ttyUSB0",
            "baudrate": 500000,
            "wheel_radius": 0.15,
            "wheel_separation": 0.60,
            "max_wheel_speed": 20.0,
            "control_rate": 20.0,
            "cmd_vel_timeout": 0.5,
            "cmd_vel_topic": "cmd_vel",
            "motor_state_topic": "motor_state",
            "wheel_command_topic": "wheel_command",
            "status_period": 1.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))

        def value(name):
            return self.get_parameter(name).value

        rate = float(value("control_rate"))
        if not math.isfinite(rate) or rate < 5.0:
            raise ValueError("control_rate must be >= 5 Hz (firmware watchdog: 250 ms)")
        status_period = float(value("status_period"))
        if not math.isfinite(status_period) or status_period < 0.0:
            raise ValueError("status_period must be >= 0; 0 disables periodic logs")
        self._commands = VelocityCommand(
            wheel_radius=float(value("wheel_radius")),
            wheel_separation=float(value("wheel_separation")),
            max_wheel_speed=float(value("max_wheel_speed")),
            timeout=float(value("cmd_vel_timeout")),
        )
        self._cmd_rx = 0
        self._cmd_rejected = 0
        self._last_feedback = None
        self._last_tx = (0.0, 0.0)
        self._last_rejection_log = -math.inf
        self._state_pub = self.create_publisher(
            Float64MultiArray, str(value("motor_state_topic")), 100
        )
        self._command_pub = self.create_publisher(
            Float64MultiArray, str(value("wheel_command_topic")), 10
        )
        # A best-effort subscriber can receive both reliable and best-effort
        # publishers. Depth 1 avoids replaying a queue of stale motion commands.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._subscription = self.create_subscription(
            Twist, str(value("cmd_vel_topic")), self._on_cmd_vel, qos
        )
        self._base = LowTempDriver(
            port=str(value("port")), baudrate=value("baudrate"),
            max_wheel_speed=self._commands.limit, serial_factory=serial_factory,
        )
        # Serial watchdog servicing must continue even when /clock is paused.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate, self._update, clock=self._steady_clock)
        self._status_timer = (self.create_timer(status_period, self._log_status,
                                               clock=self._steady_clock)
                              if status_period else None)
        self.get_logger().info(
            f"Serial connected: {value('port')} baudrate={value('baudrate')}; "
            f"radius={self._commands.radius} m separation={self._commands.separation} m; "
            f"TX={rate:g} Hz max_wheel_speed={self._commands.limit:g} rad/s"
        )
        self.get_logger().info(
            f"Input: {self._subscription.topic_name} [geometry_msgs/msg/Twist, best_effort]. "
            "Publish continuously at 10 Hz or faster. No host startup cap/ramp; "
            "feedback is optional. Starting this node alone does not request motion."
        )

    def _on_cmd_vel(self, message):
        self._cmd_rx += 1
        now = self._now()
        try:
            self._commands.accept(message.linear.x, message.angular.z, now)
        except ValueError as error:
            self._cmd_rejected += 1
            if now - self._last_rejection_log >= 1.0:
                self.get_logger().warning(f"{error}; zero target scheduled")
                self._last_rejection_log = now

    def _update(self):
        try:
            now = self._now()
            previous_reason = self._commands.reason
            target = self._commands.target(now)
            self._base.set_wheel_speeds(*target)
            self._last_tx = target
            self._command_pub.publish(Float64MultiArray(data=list(target)))
            if previous_reason != self._commands.reason and self._commands.reason == "cmd_vel_timeout":
                self.get_logger().warning("cmd_vel timed out; zero target sent")
            for state in self._base.read_states():
                self._last_feedback = self._now()
                self._state_pub.publish(Float64MultiArray(data=list(state)))
        except (OSError, RuntimeError, ValueError) as error:
            self._timer.cancel()
            self.get_logger().error(f"Serial communication failed: {error}; exiting")
            try:
                self._base.stop()
            except (OSError, RuntimeError, ValueError):
                pass
            raise SerialLinkLost(str(error)) from error

    def _log_status(self):
        now = self._now()

        def age(stamp):
            return "never" if stamp is None else f"{max(0.0, now - stamp):.2f}s"

        self.get_logger().info(
            f"IO cmd_rx={self._cmd_rx} rejected={self._cmd_rejected} "
            f"cmd_age={age(self._commands.last_command)} "
            f"TX_wheels=({self._last_tx[0]:.3f},{self._last_tx[1]:.3f})rad/s "
            f"tx_packets={self._base.tx_packets} rx_bytes={self._base.rx_bytes} "
            f"motor_packets={self._base.parser.motor_packets} "
            f"bad_frames={self._base.parser.bad_frames} "
            f"feedback_age={age(self._last_feedback)} reason={self._commands.reason}"
        )

    def close(self):
        self._timer.cancel()
        if self._status_timer is not None:
            self._status_timer.cancel()
        try:
            self._base.close()
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().warning(f"Failed to send final zero target: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = LowTempDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except SerialLinkLost:
        code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
