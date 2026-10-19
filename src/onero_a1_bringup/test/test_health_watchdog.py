"""Offline checks for feedback recovery while controller switching is pending."""
import time
from types import SimpleNamespace
import unittest

from onero_a1_bringup.health_watchdog import BridgeWatchdog


class WatchdogHarness:
    STATE_STALE = BridgeWatchdog.STATE_STALE
    STATE_RECOVERED = BridgeWatchdog.STATE_RECOVERED
    _on_state = BridgeWatchdog._on_state
    _on_switch_done = BridgeWatchdog._on_switch_done
    _sync_controller = BridgeWatchdog._sync_controller

    def __init__(self):
        self._status = self.STATE_STALE
        self._last_msg_time = None
        self._msg_count = 0
        self._timeout = 1.
        self._auto_deactivate = self._auto_reactivate = True
        self._deactivated_by_us = False
        self._switch_pending = True
        self._controller = 'arm_controller'
        self.requests = []

    def get_logger(self):
        return SimpleNamespace(warn=lambda _: None, error=lambda _: None)

    def _switch(self, activate):
        assert not self._switch_pending
        self.requests.append(activate)
        self._switch_pending = True

    def complete(self, activate):
        response = SimpleNamespace(result=lambda: SimpleNamespace(ok=True))
        self._on_switch_done(response, activate)


class WatchdogKeyChecks(unittest.TestCase):
    def test_feedback_recovers_before_deactivation_response(self):
        watchdog = WatchdogHarness()
        watchdog._on_state(None)
        self.assertEqual(watchdog.requests, [])
        watchdog.complete(activate=False)
        self.assertEqual(watchdog.requests, [True])
        watchdog.complete(activate=True)
        self.assertFalse(watchdog._deactivated_by_us)
        self.assertFalse(watchdog._switch_pending)

    def test_feedback_recovers_after_deactivation_response(self):
        watchdog = WatchdogHarness()
        watchdog.complete(activate=False)
        self.assertEqual(watchdog.requests, [])
        watchdog._on_state(None)
        self.assertEqual(watchdog.requests, [True])

    def test_feedback_becomes_stale_during_activation(self):
        watchdog = WatchdogHarness()
        watchdog._deactivated_by_us = True
        watchdog._last_msg_time = time.monotonic()-2.
        watchdog.complete(activate=True)
        self.assertEqual(watchdog.requests, [False])
        watchdog.complete(activate=False)
        self.assertTrue(watchdog._deactivated_by_us)
        self.assertFalse(watchdog._switch_pending)

    def test_auto_reactivation_remains_optional(self):
        watchdog = WatchdogHarness()
        watchdog._auto_reactivate = False
        watchdog._on_state(None)
        watchdog.complete(activate=False)
        self.assertTrue(watchdog._deactivated_by_us)
        self.assertFalse(watchdog.requests)


if __name__ == '__main__':
    unittest.main()
