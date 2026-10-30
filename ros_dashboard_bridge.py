#!/usr/bin/env python3
"""Loopback-only ROS 2 dashboard. No rosbridge/web dependencies are required."""
import argparse
from collections import deque
import json
import math
import mimetypes
from pathlib import Path
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


class ControlError(ValueError):
    pass


class ConsoleCore:
    """Thread-safe telemetry and explicit services; no driving commands are published."""
    def __init__(self, clock=time.monotonic, radius=.12, separation=.70, lift_max=0.):
        self.clock = clock
        self.lock = threading.RLock()
        self.radius, self.separation, self.lift_max = radius, separation, lift_max
        self.motor = self.applied = self.command = self.lift = self.driver = None
        self.received = {}
        self.ros_alive = False
        self.publisher_count = None
        self.graph_at = None
        self.services = {}
        self.jobs = deque()
        self.results = deque(maxlen=30)
        self.inflight_services = {}
        self.events = deque(maxlen=150)
        self.event_id = 0
        self.topics = {}

    def event(self, text, level="info"):
        self.event_id += 1
        self.events.appendleft(dict(id=self.event_id, time=time.time(), text=text, level=level))

    def age(self, topic):
        return None if topic not in self.received else max(0., self.clock() - self.received[topic])

    def update(self, topic, value):
        with self.lock:
            if topic in ("motor", "applied", "command", "lift"):
                expected = {"motor": 6, "applied": 2, "command": 2, "lift": 3}[topic]
                if len(value) != expected or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
                    return False
            if topic == "driver":
                if not isinstance(value, dict) or not isinstance(value.get("state"), str):
                    return False
                old = self.driver or {}
                if (old.get("state"), old.get("fault")) != (value.get("state"), value.get("fault")):
                    self.event("드라이버 · " + value["state"] + (" / " + str(value.get("fault")) if value.get("fault") else ""),
                               "error" if value.get("fault") else "info")
            setattr(self, topic, value)
            self.received[topic] = self.clock()
            return True

    def feedback_required(self):
        # Older drivers remain strict. Only the driver's explicit boolean opts out.
        return not (self.driver and self.driver.get("motor_feedback_required") is False)

    def motor_fresh(self):
        return self.motor is not None and self.age("motor") is not None and self.age("motor") <= .5

    def reason(self):
        if not self.ros_alive:
            return "ROS 연결을 기다리고 있어요"
        if self.age("driver") is None or self.age("driver") > 1.2:
            return "드라이버 상태가 없거나 오래되었어요"
        if self.driver.get("fault") or self.driver.get("state") == "fault":
            return "드라이버 정지 유지 / 오류를 먼저 복구하세요"
        if self.driver.get("serial_connected") is not True:
            return "드라이버의 시리얼 연결을 확인하세요"
        if self.feedback_required() and (not self.motor_fresh()
                or self.driver.get("motor_feedback_fresh") is not True):
            return "모터 피드백이 없거나 오래되었어요"
        return ""

    def _queue_service(self, name):
        for job_id, existing in self.inflight_services.items():
            if existing == name:
                return job_id
        job = dict(id=secrets.token_hex(8), kind="service", name=name, created=self.clock())
        self.jobs.append(job)
        self.inflight_services[job["id"]] = name
        self.results.appendleft(dict(id=job["id"], name=name, status="pending", message="응답 대기"))
        return job["id"]

    def request_service(self, name):
        with self.lock:
            if name not in ("driver_stop", "driver_reset", "lift_set_home", "lift_stop"):
                raise ControlError("unknown_service")
            if not self.services.get(name):
                raise ControlError("ROS 서비스가 연결되지 않았어요")
            if name == "lift_set_home":
                self._check_lift()
            if name in self.inflight_services.values():
                raise ControlError("같은 요청의 응답을 기다리고 있어요")
            if len(self.jobs) >= 20:
                raise ControlError("too_many_requests")
            return self._queue_service(name)

    def _check_lift(self):
        if self.lift_max <= 0 or self.lift is None or self.age("lift") > .5:
            raise ControlError("리프트 설정과 피드백을 확인하세요")
        if self.motor is None or self.age("motor") > .5 or max(abs(self.motor[0]), abs(self.motor[3])) > .1:
            raise ControlError("주행 바퀴가 정지한 뒤 조작하세요")
        if self.driver is None or self.age("driver") > 1.2 or self.driver.get("fault"):
            raise ControlError("드라이버 상태를 확인하세요")
        flags = int(self.lift[2])
        if flags & 8 or not flags & 1:
            raise ControlError("리프트 센서 상태를 확인하세요")

    def lift_target(self, height):
        with self.lock:
            self._check_lift()
            if not isinstance(height, (float, int)) or isinstance(height, bool) or not math.isfinite(height) or not 0 <= height <= self.lift_max:
                raise ControlError("목표 높이가 허용 범위를 벗어났어요")
            if not int(self.lift[2]) & 32 or int(self.lift[2]) & 16:
                raise ControlError("HOME 완료 후 목표 높이를 입력하세요")
            if len(self.jobs) >= 20:
                raise ControlError("too_many_requests")
            job = dict(id=secrets.token_hex(8), kind="lift_target", height=float(height), created=self.clock())
            self.jobs.append(job)
            self.results.appendleft(dict(id=job["id"], name="lift_target", status="pending", message="전송 대기"))
            return job["id"]

    def finish(self, job_id, success, message, unknown=False):
        with self.lock:
            if not unknown:
                self.inflight_services.pop(job_id, None)
            for result in self.results:
                if result["id"] == job_id:
                    result.update(status="unknown" if unknown else ("success" if success else "error"), message=str(message))
                    self.event(result["name"] + " · " + str(message), "info" if success else "error")
                    break

    def snapshot(self):
        with self.lock:
            ages = {name: self.age(name) for name in ("motor", "driver", "applied", "command", "lift")}
            return dict(mode="live", ros_alive=self.ros_alive, driver=self.driver,
                        motor=self.motor, applied=self.applied, command=self.command, lift=self.lift, ages=ages,
                        blocked_reason=self.reason(), services=self.services.copy(),
                        publishers_fresh=self.graph_at is not None and self.clock() - self.graph_at <= 2.,
                        publisher_count=self.publisher_count, topics=self.topics.copy(),
                        config=dict(wheel_radius=self.radius, wheel_separation=self.separation, lift_max_height=self.lift_max),
                        events=list(self.events), results=[dict(r) for r in self.results])


def make_ros_node(core):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Float64, Float64MultiArray
    from std_srvs.srv import Trigger

    class BridgeNode(Node):
        def __init__(self):
            super().__init__("mf_robot_console")
            self.lift_pub = self.create_publisher(Float64, "lift_target", 1)
            telemetry = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
            self.telemetry_subs = {
                "cmd_vel": self.create_subscription(Twist, "cmd_vel", lambda m: core.update("command", [m.linear.x, m.angular.z]), telemetry),
                "motor_state": self.create_subscription(Float64MultiArray, "motor_state", lambda m: core.update("motor", list(m.data)), telemetry),
                "lift_state": self.create_subscription(Float64MultiArray, "lift_state", lambda m: core.update("lift", list(m.data)), telemetry),
                "cmd_vel_applied": self.create_subscription(Twist, "cmd_vel_applied", lambda m: core.update("applied", [m.linear.x, m.angular.z]), telemetry),
                "driver_status": self.create_subscription(String, "driver_status", self.on_status, telemetry),
            }
            self.clients = {name: self.create_client(Trigger, name) for name in ("driver_stop", "driver_reset", "lift_set_home", "lift_stop")}
            self.pending = {}
            self.graph_time = 0.
            self.timer = self.create_timer(.05, self.update)
            with core.lock:
                core.topics = {name: sub.topic_name for name, sub in self.telemetry_subs.items()}
                core.topics["lift_target"] = self.lift_pub.topic_name
                core.ros_alive = True
                core.event("ROS2 모니터링 시작 · 주행 명령은 외부에서 발행")

        def on_status(self, msg):
            try:
                core.update("driver", json.loads(msg.data))
            except (ValueError, TypeError):
                pass

        def update(self):
            now = core.clock()
            if now - self.graph_time >= .5:
                with core.lock:
                    core.publisher_count = self.count_publishers(self.telemetry_subs["cmd_vel"].topic_name)
                    core.graph_at = now
                    core.services = {name: client.service_is_ready() for name, client in self.clients.items()}
                self.graph_time = now
            with core.lock:
                jobs = list(core.jobs)
                core.jobs.clear()
            for job in jobs:
                if now - job["created"] > 1.:
                    core.finish(job["id"], False, "요청이 지연되어 전송하지 않았어요")
                    continue
                if job["kind"] == "lift_target":
                    try:
                        with core.lock:
                            core._check_lift()
                            if not int(core.lift[2]) & 32 or int(core.lift[2]) & 16:
                                raise ControlError("HOME 상태를 확인하세요")
                        msg = Float64()
                        msg.data = job["height"]
                        self.lift_pub.publish(msg)
                        core.finish(job["id"], True, "목표를 발행했어요. 도달 여부는 피드백으로 확인하세요")
                    except ControlError as error:
                        core.finish(job["id"], False, error)
                    continue
                client = self.clients[job["name"]]
                if not client.service_is_ready():
                    core.finish(job["id"], False, "서비스 연결이 끊겼어요")
                    continue
                future = client.call_async(Trigger.Request())
                self.pending[job["id"]] = (future, now, False)
            for job_id, (future, sent, reported) in list(self.pending.items()):
                if future.done():
                    try:
                        result = future.result()
                        core.finish(job_id, result.success, result.message)
                    except Exception as error:
                        core.finish(job_id, False, error)
                    del self.pending[job_id]
                elif now - sent > 3. and not reported:
                    core.finish(job_id, False, "응답 지연 · 실행 여부 미확인. 자동 재시도하지 않아요", unknown=True)
                    self.pending[job_id] = (future, sent, True)

    return BridgeNode()


def handler_factory(core, static_dir, port, csrf_token):
    origins = {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}
    hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
    allowed = {"/": "index.html", "/index.html": "index.html", "/app.js": "app.js", "/styles.css": "styles.css", "/favicon.svg": "favicon.svg"}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, status, data, content_type="application/json; charset=utf-8"):
            body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if not isinstance(data, bytes) else data
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def trusted(self, write=False):
            if self.headers.get("Host") not in hosts:
                return False
            if self.headers.get("Origin") and self.headers.get("Origin") not in origins:
                return False
            if self.headers.get("Sec-Fetch-Site") == "cross-site":
                return False
            return not write or secrets.compare_digest(self.headers.get("X-Console-Token", ""), csrf_token)

        def do_GET(self):
            if not self.trusted():
                return self.respond(403, {"error": "origin_rejected"})
            path = urlsplit(self.path).path
            if path == "/api/session":
                return self.respond(200, {"token": csrf_token, "mode": "live"})
            if path == "/api/state":
                return self.respond(200, core.snapshot())
            if path not in allowed:
                return self.respond(404, {"error": "not_found"})
            file = static_dir / allowed[path]
            if not file.is_file():
                return self.respond(404, {"error": "missing_static_file"})
            return self.respond(200, file.read_bytes(), mimetypes.guess_type(str(file))[0] or "application/octet-stream")

        def do_POST(self):
            if not self.trusted(write=True):
                return self.respond(403, {"error": "origin_or_token_rejected"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096 or self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    return self.respond(400, {"error": "invalid_body"})
                self.connection.settimeout(2)
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ControlError("invalid_body")
                path = urlsplit(self.path).path
                if path == "/api/service":
                    return self.respond(202, {"job_id": core.request_service(data.get("name"))})
                if path == "/api/lift-target":
                    return self.respond(202, {"job_id": core.lift_target(data.get("height"))})
                return self.respond(404, {"error": "not_found"})
            except (ValueError, TypeError, OSError) as error:
                return self.respond(400, {"error": str(error)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--wheel-radius", type=float, default=.12)
    parser.add_argument("--wheel-separation", type=float, default=.70)
    parser.add_argument("--lift-max-height", type=float, default=0.)
    args, ros_args = parser.parse_known_args()
    if not 1024 <= args.port <= 65535 or any(not math.isfinite(v) or v <= 0 for v in (args.wheel_radius, args.wheel_separation)) or not math.isfinite(args.lift_max_height) or args.lift_max_height < 0:
        parser.error("invalid port/geometry/height")
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        parser.error("ROS2 환경을 먼저 불러오세요: source /opt/ros/humble/setup.bash")
    core = ConsoleCore(radius=args.wheel_radius, separation=args.wheel_separation, lift_max=args.lift_max_height)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_factory(core, Path(__file__).parent / "dist", args.port, secrets.token_urlsafe(32)))
    server.daemon_threads = True
    rclpy.init(args=ros_args)
    node = make_ros_node(core)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"MF Robot Console: http://127.0.0.1:{args.port}")
    print("주행 모니터링 / 동일 PC 전용 / 드라이버와 teleop을 별도 실행하세요")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with core.lock:
            core.ros_alive = False
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
