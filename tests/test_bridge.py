"""No ROS/hardware access: command-lease and HTTP boundary regression tests."""
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
        self.ready()

    def ready(self):
        c = self.core
        c.ros_alive = True
        c.publisher_count = 1
        c.graph_at = self.now
        c.services = dict(driver_stop=True, driver_reset=True, lift_set_home=True, lift_stop=True)
        c.update("motor", [0.] * 6)
        c.update("applied", [0., 0.])
        c.update("driver", dict(state="ready", fault="", serial_connected=True, motor_feedback_fresh=True))
        c.update("lift", [0., 100., 33.])

    def arm(self):
        return self.core.arm("browser-client-one")

    def test_idle_dashboard_does_not_publish_zeros(self):
        self.assertIsNone(self.core.tick())

    def test_held_command_then_release(self):
        token = self.arm()
        self.core.command(token, 1, .1, .2)
        self.assertEqual(self.core.tick(), (.1, .2))
        self.core.command(token, 2, 0., 0.)
        self.assertEqual(self.core.tick(), (0., 0.))

    def test_browser_lost_lease_sends_one_zero(self):
        token = self.arm()
        self.core.command(token, 1, .1, 0.)
        self.now += .36
        self.assertEqual(self.core.tick(), (0., 0.))
        self.assertIsNone(self.core.tick())
        self.assertIsNone(self.core.controller)

    def test_expired_delayed_command_cannot_restart(self):
        token = self.arm()
        self.core.command(token, 1, .1, 0.)
        self.now += .36
        with self.assertRaises(ControlError):
            self.core.command(token, 2, .1, 0.)
        self.assertIsNone(self.core.controller)

    def test_command_reordering_cannot_override_stop(self):
        token = self.arm()
        self.core.command(token, 20, 0., 0.)
        with self.assertRaises(ControlError):
            self.core.command(token, 19, .3, .5)
        self.assertEqual(self.core.tick(), (0., 0.))

    def test_previous_session_cannot_override_new_session(self):
        previous = self.arm()
        self.core.disarm()
        current = self.arm()
        with self.assertRaises(ControlError):
            self.core.command(previous, 100, .1, 0.)
        self.core.command(current, 1, 0., 0.)

    def test_other_tab_cannot_take_over(self):
        self.arm()
        with self.assertRaises(ControlError):
            self.core.arm("another-client-id")
        self.core.disarm(epoch="wrong-epoch")
        self.assertIsNotNone(self.core.controller)

    def test_stale_motor_blocks_arm(self):
        self.now += .51
        with self.assertRaises(ControlError):
            self.arm()

    def optional_feedback(self):
        self.core.motor = None
        self.core.received.pop("motor", None)
        self.core.update("driver", dict(state="ready", fault="", serial_connected=True,
                                       motor_feedback_required=False, motor_feedback_fresh=False))

    def test_optional_driver_can_arm_and_command_without_motor(self):
        self.optional_feedback()
        token = self.arm()
        self.core.command(token, 1, .1, 0.)
        self.assertEqual(self.core.tick(), (.1, 0.))
        self.assertIsNone(self.core.snapshot()["motor"])
        self.assertIsNone(self.core.snapshot()["ages"]["motor"])

    def test_optional_stale_motor_no_longer_blocks(self):
        self.optional_feedback()
        self.core.update("motor", [1., 0., 0., 1., 0., 0.])
        self.now += .6
        self.core.update("driver", self.core.driver)
        self.assertTrue(self.arm())

    def test_optional_driver_stale_status_still_blocks(self):
        self.optional_feedback()
        self.now += 1.3
        with self.assertRaises(ControlError):
            self.arm()

    def test_optional_serial_failure_still_blocks(self):
        self.optional_feedback()
        self.core.driver["serial_connected"] = False
        with self.assertRaises(ControlError):
            self.arm()

    def test_optional_fresh_moving_wheels_still_block(self):
        self.optional_feedback()
        self.core.update("motor", [1., 0., 0., 1., 0., 0.])
        with self.assertRaises(ControlError):
            self.arm()

    def test_optional_browser_timeout_still_stops(self):
        self.optional_feedback()
        token = self.arm()
        self.core.command(token, 1, .1, 0.)
        self.now += .36
        self.assertEqual(self.core.tick(), (0., 0.))
        self.assertIsNone(self.core.controller)

    def test_feedback_optout_requires_exact_boolean_false(self):
        self.optional_feedback()
        for value in (True, "false", None, 0):
            self.core.driver["motor_feedback_required"] = value
            with self.assertRaises(ControlError):
                self.arm()

    def test_optional_does_not_disable_lift_feedback_checks(self):
        self.optional_feedback()
        with self.assertRaises(ControlError):
            self.core.lift_target(.1)

    def test_stale_driver_blocks_arm(self):
        self.now += 1.3
        self.core.update("motor", [0.] * 6)
        with self.assertRaises(ControlError):
            self.arm()

    def test_fault_blocks_arm(self):
        self.core.update("driver", dict(state="fault", fault="operator_stop", serial_connected=True, motor_feedback_fresh=True))
        with self.assertRaises(ControlError):
            self.arm()

    def test_moving_wheels_block_arm(self):
        self.core.update("motor", [1., 0., 0., 1., 0., 0.])
        with self.assertRaises(ControlError):
            self.arm()

    def test_competing_publisher_latches_driver_stop_when_controlling(self):
        token = self.arm()
        self.core.command(token, 1, .1, 0.)
        self.core.publisher_count = 2
        self.assertEqual(self.core.tick(), (0., 0.))
        self.assertEqual(self.core.jobs[0]["name"], "driver_stop")
        self.assertIsNone(self.core.controller)

    def test_invalid_values_cancel_old_command(self):
        for bad in (float('nan'), float('inf'), True, '0.1', .31):
            token = self.arm()
            with self.assertRaises(ControlError):
                self.core.command(token, 1, bad, 0.)
            self.assertIsNone(self.core.controller)
            self.core.tick()

    def test_invalid_telemetry_does_not_refresh_age(self):
        self.now += .2
        self.assertFalse(self.core.update("motor", [float('nan')]*6))
        self.assertAlmostEqual(self.core.age("motor"), .2)

    def test_unavailable_stop_service_still_disarms(self):
        self.arm()
        self.core.services["driver_stop"] = False
        with self.assertRaises(ControlError):
            self.core.request_service("driver_stop")
        self.assertEqual(self.core.tick(), (0., 0.))

    def test_unknown_service_not_queued(self):
        with self.assertRaises(ControlError):
            self.core.request_service("anything")
        self.assertFalse(self.core.jobs)

    def test_service_unknown_outcome_cannot_be_retried(self):
        job = self.core.request_service("lift_set_home")
        self.core.finish(job, False, "unknown", unknown=True)
        with self.assertRaises(ControlError):
            self.core.request_service("lift_set_home")
        self.core.finish(job, True, "completed")
        self.assertTrue(self.core.request_service("lift_set_home"))

    def test_lift_requires_homed_and_range(self):
        with self.assertRaises(ControlError):
            self.core.lift_target(.6)
        self.core.update("lift", [0., 100., 1.])
        with self.assertRaises(ControlError):
            self.core.lift_target(.3)
        self.core.update("lift", [0., 100., 33.])
        self.assertTrue(self.core.lift_target(.3))

    def test_lift_rejects_moving_wheels_or_fault(self):
        self.core.update("motor", [.2, 0., 0., .2, 0., 0.])
        with self.assertRaises(ControlError):
            self.core.request_service("lift_set_home")
        self.core.update("motor", [0.] * 6)
        self.core.update("lift", [0., 100., 41.])
        with self.assertRaises(ControlError):
            self.core.lift_target(.1)

    def test_snapshot_geometry_and_json(self):
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot['config']['wheel_separation'], .7)
        json.dumps(snapshot, allow_nan=False)


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

    def test_command_rejected_without_valid_session(self):
        self.assertEqual(self.request('POST','/api/command',{'epoch':'bad','seq':1,'linear':.1,'angular':0}, {'Content-Type':'application/json','X-Console-Token':'test-csrf-token'})[0], 400)

    def test_malformed_body_rejected(self):
        self.assertEqual(self.request('POST','/api/command',[], {'Content-Type':'application/json','X-Console-Token':'test-csrf-token'})[0], 400)


if __name__ == '__main__':
    unittest.main()
