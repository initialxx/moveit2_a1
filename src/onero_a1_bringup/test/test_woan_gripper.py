"""Key offline checks: shared transport and freshness of partial joint states."""
import threading
from types import SimpleNamespace
import unittest
from pathlib import Path
import tempfile

from sensor_msgs.msg import JointState
from onero_a1_bringup.wait_for_joint_states import JointStateCache
from onero_a1_bringup.sdk_session import _DeferredBackend, GRIPPER_FEEDBACK_TIMEOUT
from onero_a1_bringup.woan_gripper import (
    ENDPOINT_TOLERANCE, STROKE, _feedback_after_command, _target_opening,
)
from onero_sdk.grippers import OneroGripper
from onero_sdk.model_registry import get_model
from onero_a1_bringup.gripper_coordinates import GripperCalibration, load_calibration, save_calibration


class WoanKeyChecks(unittest.TestCase):
    def test_moveit_endpoint_region_uses_calibrated_drive_endpoints(self):
        calibration = GripperCalibration(.07925353775867743, 1., 0., 1.)
        for angle in (0., .004771, ENDPOINT_TOLERANCE):
            opening = _target_opening(angle)
            self.assertEqual(opening, 0.)
            self.assertEqual(calibration.target_to_sdk(opening), 0.)
        for angle in (STROKE, STROKE-.004771, STROKE-ENDPOINT_TOLERANCE):
            opening = _target_opening(angle)
            self.assertEqual(opening, 1.)
            self.assertEqual(calibration.target_to_sdk(opening), 1.)
        for angle in (.01, .55, 1.09):
            opening = _target_opening(angle)
            self.assertAlmostEqual(opening, angle/STROKE)
            self.assertAlmostEqual(calibration.target_to_sdk(opening),
                                   calibration.closed+opening*(calibration.opened-calibration.closed))
        for angle in (-.001, STROKE+.001, float('nan')):
            with self.assertRaises(ValueError):
                _target_opening(angle)

    def test_missing_command_reply_uses_one_passive_query(self):
        calls = []
        old = SimpleNamespace(timestamp=1.)
        fresh = SimpleNamespace(timestamp=2.)
        def refresh():
            calls.append('passive_query')
            return fresh
        device = SimpleNamespace(state=old, refresh=refresh)
        self.assertIs(_feedback_after_command(device, 1.), fresh)
        self.assertEqual(calls, ['passive_query'])
        calls.clear()
        device.state = fresh
        self.assertIs(_feedback_after_command(device, 1.), fresh)
        self.assertFalse(calls)

    def test_passive_query_must_return_new_feedback(self):
        old = SimpleNamespace(timestamp=1.)
        device = SimpleNamespace(state=old, refresh=lambda: old)
        with self.assertRaisesRegex(RuntimeError, '未取得新反馈'):
            _feedback_after_command(device, 1.)
        def timeout():
            raise RuntimeError('Onero gripper did not respond to status query')
        device.refresh = timeout
        with self.assertRaisesRegex(RuntimeError, 'did not respond'):
            _feedback_after_command(device, 1.)

    def test_calibration_file_is_bound_to_device_and_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'calibration.json'
            identity = {'model':'A1R', 'channel':'slcan0', 'simulate':False}
            calibration = GripperCalibration(.0310123, 1.)
            self.assertIsNone(load_calibration(path, **identity))
            save_calibration(path, calibration, **identity, endpoints={})
            self.assertEqual(load_calibration(path, **identity), calibration)
            with self.assertRaises(ValueError):
                load_calibration(path, model='A1R', channel='slcan0', simulate=True)
            with self.assertRaises(ValueError):
                load_calibration(path, model='A1R', channel='slcan1', simulate=False)
            self.assertFalse(list(path.parent.glob('*.tmp')))

    def test_measured_closed_endpoint_maps_to_zero_without_changing_sdk(self):
        closed = .034113479296126376/1.1
        calibration = GripperCalibration(closed, 1.)
        self.assertEqual(calibration.from_sdk(closed), 0.)
        self.assertEqual(calibration.to_sdk(0.), closed)
        self.assertEqual(calibration.to_sdk(1.), 1.)
        self.assertAlmostEqual(calibration.from_sdk(calibration.to_sdk(.5)), .5)
        for endpoints in [(1., 0.), (.5, .5), (float('nan'), 1.), (0., 1.1)]:
            with self.assertRaises(ValueError): GripperCalibration(*endpoints)

    def test_partial_messages_do_not_refresh_stale_arm_joints(self):
        cache = JointStateCache(['arm', 'finger'])
        arm = JointState(name=['arm'], position=[.3]); arm.header.stamp.sec = 10
        grip = JointState(name=['finger'], position=[.5]); grip.header.stamp.sec = 10
        self.assertIsNone(cache.update(arm, 10.))
        self.assertEqual(cache.update(grip, 10.), {'arm': .3, 'finger': .5})
        arm.position = [float('nan')]
        self.assertIsNone(cache.update(arm, 10.))
        arm.position = [.3]
        cache.update(arm, 10.)
        grip.header.stamp.sec = 12
        self.assertIsNone(cache.update(grip, 12.))

    def test_gripper_uses_arm_owned_bus_and_only_queries_on_startup(self):
        class Bus:
            def __init__(self): self.frames = []; self.timeouts = []
            def send(self, can_id, data): self.frames.append((can_id, data))
            def recv_until(self, _id, timeout):
                self.timeouts.append(timeout)
                p = round(238*257 + .4*(37*257-238*257))
                return SimpleNamespace(data=bytes([8, p>>8, p&255, 0x80, 8, 0]))
        bus = Bus(); created = []
        class Backend:
            def __init__(self, **_kwargs): self.lock = threading.RLock(); created.append(self)
            def bind_can_device(self, device): device.bind_transport(bus, self.lock)
            def enable(self): return 'measured-arm-state'
            def close(self, disable=False): pass
        gripper = OneroGripper(model='A1R', feedback_timeout=GRIPPER_FEEDBACK_TIMEOUT)
        backend = _DeferredBackend(get_model('A1R'), 'offline-woan-test', 15., None, None, Backend, gripper)
        try:
            self.assertEqual(backend.enable(), 'measured-arm-state')
            self.assertEqual(backend.enable(), 'measured-arm-state')
            state = gripper.refresh()
            self.assertEqual(len(created), 1)
            self.assertAlmostEqual(state.position, .4, places=4)
            self.assertFalse(gripper.enabled)
            self.assertEqual(len(bus.frames), 1)
            self.assertEqual(bus.frames[0][0], 0x7FF)
            self.assertEqual(bus.timeouts, [GRIPPER_FEEDBACK_TIMEOUT])
        finally:
            backend.close()


if __name__ == '__main__':
    unittest.main()
