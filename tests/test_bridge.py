"""No ROS/hardware access: passive driving monitor, explicit services and HTTP checks."""
import http.client
import json
from pathlib import Path
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_dashboard_bridge import ConsoleCore, ControlError, handler_factory


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.core = ConsoleCore(clock=lambda: self.now, lift_max=.5)
        c = self.core
        c.ros_alive = True
        c.publisher_count = 1
        c.graph_at = self.now
        c.services = dict(driver_stop=True, driver_reset=True, lift_set_home=True, lift_stop=True)
        c.update("motor", [0.] * 6)
        c.update("applied", [0., 0.])
        c.update("driver", dict(state="ready", fault="", serial_connected=True, motor_feedback_fresh=True))
        c.update("lift", [0., 100., 33.])

    def test_external_command_is_telemetry_without_manual_speed_cap(self):
        self.assertTrue(self.core.update("command", [.8, 1.2]))
        self.assertEqual(self.core.snapshot()['command'], [.8, 1.2])
        self.assertFalse(self.core.jobs)

    def test_invalid_command_does_not_refresh_age(self):
        self.core.update("command", [.1, 0.])
        self.now += .2
        for bad in ([float('nan'), 0.], [True, 0.], [0.], ['0.1', 0.]):
            self.assertFalse(self.core.update("command", bad))
        self.assertAlmostEqual(self.core.snapshot()['ages']['command'], .2)

    def test_absent_and_old_commands_are_distinguishable_from_zero(self):
        self.assertIsNone(self.core.snapshot()['ages']['command'])
        self.core.update("command", [0., 0.])
        self.now += 2.
        self.assertEqual(self.core.snapshot()['command'], [0., 0.])
        self.assertEqual(self.core.snapshot()['ages']['command'], 2.)
        self.assertFalse(self.core.jobs)

    def test_multiple_publishers_and_feedback_loss_do_not_queue_stop(self):
        self.core.publisher_count = 3
        self.now += 3.
        for _ in range(5):
            self.core.snapshot()
        self.assertFalse(self.core.jobs)
        self.assertFalse(self.core.inflight_services)

    def test_optional_feedback_status(self):
        self.core.motor = None
        self.core.received.pop('motor')
        self.core.update('driver', dict(state='ready', fault='', serial_connected=True, motor_feedback_required=False))
        self.assertEqual(self.core.reason(), '')
        self.assertIsNone(self.core.snapshot()['motor'])

    def test_required_feedback_status(self):
        self.now += .6
        self.assertIn('피드백', self.core.reason())

    def test_stop_is_requested_only_explicitly(self):
        self.core.snapshot()
        self.assertFalse(self.core.jobs)
        self.core.request_service('driver_stop')
        self.assertEqual(self.core.jobs[0]['name'], 'driver_stop')
        self.assertEqual(len(self.core.jobs), 1)

    def test_unavailable_service_is_not_queued(self):
        self.core.services['driver_stop'] = False
        with self.assertRaises(ControlError):
            self.core.request_service('driver_stop')
        self.assertFalse(self.core.jobs)

    def test_unknown_service_not_queued(self):
        with self.assertRaises(ControlError):
            self.core.request_service('anything')
        self.assertFalse(self.core.jobs)

    def test_service_unknown_outcome_cannot_be_retried(self):
        job = self.core.request_service('driver_stop')
        self.core.finish(job, False, 'unknown', unknown=True)
        with self.assertRaises(ControlError):
            self.core.request_service('driver_stop')
        self.core.finish(job, True, 'completed')
        self.assertTrue(self.core.request_service('driver_stop'))

    def test_lift_requires_homed_range_and_stopped_wheels(self):
        for bad in (.6, -.1, float('nan'), True):
            with self.assertRaises(ControlError):
                self.core.lift_target(bad)
        self.assertTrue(self.core.lift_target(.1))
        self.core.update('motor', [1., 0., 0., 1., 0., 0.])
        with self.assertRaises(ControlError):
            self.core.request_service('lift_set_home')
        self.core.update('motor', [0.] * 6)
        self.core.update('lift', [0., 100., 1.])
        with self.assertRaises(ControlError):
            self.core.lift_target(.1)
        self.core.update('lift', [0., 100., 41.])
        with self.assertRaises(ControlError):
            self.core.request_service('lift_set_home')

    def test_snapshot_geometry_and_json(self):
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot['config']['wheel_separation'], .7)
        self.assertTrue(snapshot['publishers_fresh'])
        self.assertNotIn('manual_allowed', snapshot)
        json.dumps(snapshot, allow_nan=False)


class ROSInterfaceTests(unittest.TestCase):
    """Exercise the adapter against fake ROS endpoints, with no real robot."""
    def setUp(self):
        import types
        from unittest.mock import patch
        from ros_dashboard_bridge import make_ros_node
        self.now = 100.
        self.core = ConsoleCore(clock=lambda: self.now)
        self.publishers = []
        self.clients = []
        outer = self

        class FakeNode:
            def __init__(self, name):
                self.publisher_count = 2
            def create_publisher(self, kind, topic, qos):
                pub = types.SimpleNamespace(topic_name='/robot/' + topic, values=[])
                pub.publish = pub.values.append
                outer.publishers.append((kind, topic, pub))
                return pub
            def create_subscription(self, kind, topic, callback, qos):
                return types.SimpleNamespace(topic_name='/robot/' + topic, callback=callback)
            def create_client(self, kind, name):
                client = types.SimpleNamespace(name=name, calls=[], service_is_ready=lambda: True)
                def call(request):
                    client.calls.append(request)
                    return types.SimpleNamespace(done=lambda: True, result=lambda: types.SimpleNamespace(success=True, message='ok'))
                client.call_async = call
                outer.clients.append(client)
                return client
            def create_timer(self, period, callback):
                return types.SimpleNamespace(callback=callback)
            def count_publishers(self, topic):
                outer.counted_topic = topic
                return self.publisher_count

        modules = {
            'rclpy': types.ModuleType('rclpy'),
            'rclpy.node': types.SimpleNamespace(Node=FakeNode),
            'rclpy.qos': types.SimpleNamespace(QoSProfile=lambda **kw: kw, ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1), DurabilityPolicy=types.SimpleNamespace(VOLATILE=1)),
            'geometry_msgs.msg': types.SimpleNamespace(Twist=type('Twist', (), {})),
            'std_msgs.msg': types.SimpleNamespace(String=type('String', (), {}), Float64=type('Float64', (), {}), Float64MultiArray=type('Float64MultiArray', (), {})),
            'std_srvs.srv': types.SimpleNamespace(Trigger=types.SimpleNamespace(Request=lambda: object())),
        }
        self.patch = patch.dict(sys.modules, modules)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.node = make_ros_node(self.core)

    def test_no_velocity_publisher_or_automatic_stop(self):
        for _ in range(4):
            self.node.update()
            self.now += 1.
        self.assertEqual([topic for _, topic, _ in self.publishers], ['lift_target'])
        self.assertFalse(any(pub.values for _, _, pub in self.publishers))
        self.assertFalse(any(client.calls for client in self.clients))
        self.assertEqual(self.counted_topic, '/robot/cmd_vel')
        self.assertEqual(self.core.publisher_count, 2)

    def test_external_twist_subscription_and_resolved_topic(self):
        import types
        self.node.telemetry_subs['cmd_vel'].callback(types.SimpleNamespace(linear=types.SimpleNamespace(x=.7), angular=types.SimpleNamespace(z=1.)))
        self.assertEqual(self.core.snapshot()['command'], [.7, 1.])
        self.assertEqual(self.core.topics['cmd_vel'], '/robot/cmd_vel')
        self.node.update()
        self.assertFalse(any(client.calls for client in self.clients))

    def test_explicit_stop_and_reset_services_still_work(self):
        self.node.update()
        for name in ('driver_stop', 'driver_reset'):
            self.core.request_service(name)
            self.node.update()
            self.assertEqual(len(next(c for c in self.clients if c.name == name).calls), 1)
            self.assertEqual(self.core.results[0]['status'], 'success')


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.core = ConsoleCore()
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), lambda *a, **k: None)
        cls.port = cls.server.server_port
        cls.server.RequestHandlerClass = handler_factory(cls.core, Path(__file__).resolve().parents[1]/'dist', cls.port, 'test-csrf-token')
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def request(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        payload = json.dumps(body) if body is not None else None
        conn.request(method, path, payload, headers=headers or {})
        response = conn.getresponse()
        data = response.read()
        status = response.status
        conn.close()
        return status, data

    def test_static_and_session(self):
        self.assertEqual(self.request('GET','/')[0], 200)
        status, data = self.request('GET','/api/session')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(data)['mode'], 'live')

    def test_dns_rebinding_host_rejected(self):
        self.assertEqual(self.request('GET','/api/state', headers={'Host':'attacker.example'})[0], 403)

    def test_cross_origin_rejected(self):
        self.assertEqual(self.request('GET','/api/session', headers={'Origin':'https://attacker.example'})[0], 403)

    def test_token_required(self):
        self.assertEqual(self.request('POST','/api/arm',{'client':'client-12345'}, {'Content-Type':'application/json'})[0], 403)

    def test_unknown_path_and_traversal_rejected(self):
        self.assertEqual(self.request('GET','/../ros_dashboard_bridge.py')[0], 404)
        self.assertEqual(self.request('POST','/api/exec',{}, {'Content-Type':'application/json','X-Console-Token':'test-csrf-token'})[0], 404)

    def test_driving_endpoints_are_removed_even_with_valid_csrf(self):
        for path in ('arm', 'command', 'disarm'):
            self.assertEqual(self.request('POST','/api/' + path, {'linear':.1,'angular':0}, {'Content-Type':'application/json','X-Console-Token':'test-csrf-token'})[0], 404)
        self.assertFalse(self.core.jobs)

    def test_malformed_body_rejected(self):
        self.assertEqual(self.request('POST','/api/service',[], {'Content-Type':'application/json','X-Console-Token':'test-csrf-token'})[0], 400)


if __name__ == '__main__':
    unittest.main()
