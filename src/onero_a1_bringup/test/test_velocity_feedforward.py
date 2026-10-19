"""Focused, hardware-free checks for velocity command forwarding."""

import unittest

from onero_a1_bringup.velocity_feedforward import VelocityFeedforward


class VelocityFeedforwardTest(unittest.TestCase):
    def setUp(self):
        self.names = [f"joint{i}" for i in range(7)]
        self.guard = VelocityFeedforward(
            self.names, enabled=True, timeout=0.1)

    def capture(self, **changes):
        args = dict(names=self.names, positions=[0.0] * 7,
                    velocities=[0.1] * 7, stamp_ns=1_000_000_000,
                    now_s=2.0, now_ros_ns=1_000_000_000)
        args.update(changes)
        self.guard.capture(**args)

    def sample(self, **changes):
        args = dict(now_s=2.01, now_ros_ns=1_010_000_000)
        args.update(changes)
        return self.guard.sample(**args)

    def test_maps_same_frame_by_name_for_all_seven_joints(self):
        self.capture(names=self.names[::-1], velocities=list(range(1, 8)))
        raw, applied, reason = self.sample()
        self.assertEqual(raw, [7, 6, 5, 4, 3, 2, 1])
        self.assertEqual(applied, raw)
        self.assertEqual(reason, "active")

    def test_off_still_records_input(self):
        self.guard.enabled = False
        self.capture()
        raw, applied, reason = self.sample()
        self.assertEqual(raw, [0.1] * 7)
        self.assertEqual(applied, [0.0] * 7)
        self.assertEqual(reason, "disabled")

    def test_new_bad_frame_clears_previous_velocity(self):
        cases = [dict(velocities=[]), dict(velocities=[0.1]),
                 dict(velocities=[float("nan")] * 7),
                 dict(positions=[float("inf")] * 7),
                 dict(names=self.names[:-1]), dict(names=[self.names[0]] * 7)]
        for case in cases:
            with self.subTest(case=case):
                self.guard.clear("reset", reset_order=True)
                self.capture()
                self.capture(stamp_ns=1_001_000_000, **case)
                self.assertEqual(self.sample()[1], [0.0] * 7)

    def test_stale_or_out_of_order_frames_do_not_keep_velocity(self):
        self.capture()
        self.capture(stamp_ns=999_000_000)
        self.assertEqual(self.sample()[2], "out_of_order")
        self.guard.clear("reset", reset_order=True)
        self.capture(stamp_ns=800_000_000)
        self.assertEqual(self.sample()[2], "stale_header")
        self.capture(stamp_ns=0)
        self.assertEqual(self.sample()[1], [0.0] * 7)

    def test_future_frame_does_not_poison_next_valid_frame(self):
        self.capture(stamp_ns=9_000_000_000)
        self.assertEqual(self.sample()[2], "stale_header")
        self.capture()
        self.assertEqual(self.sample()[2], "active")

    def test_receipt_and_source_timeouts(self):
        self.capture()
        self.assertEqual(self.sample(now_s=2.2)[1], [0.0] * 7)
        self.assertEqual(self.sample(now_ros_ns=1_200_000_000)[1], [0.0] * 7)

    def test_stop_hold_and_reenable(self):
        self.capture()
        self.capture(stamp_ns=1_001_000_000, velocities=[1e-17] * 7)
        self.assertEqual(self.sample()[1], [0.0] * 7)
        self.guard.clear("fault_hold")
        self.assertEqual(self.sample()[1], [0.0] * 7)
        self.guard.clear("reenabled", reset_order=True)
        self.assertEqual(self.sample()[1], [0.0] * 7)
        self.capture()
        self.assertEqual(self.sample()[2], "active")

    def test_invalid_configuration(self):
        for scale in ([1.0], [float("nan")] * 7, [1.1] * 7, [-0.1] * 7,
                      [1, 1, 1, 1, 0, 0, 0], [0.5] * 7):
            with self.assertRaises(ValueError):
                VelocityFeedforward(self.names, scale=scale)
        for timeout in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                VelocityFeedforward(self.names, scale=[1.0] * 7, timeout=timeout)


if __name__ == "__main__":
    unittest.main()
