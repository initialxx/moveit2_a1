"""Mode selection must resolve before any hardware startup."""
import unittest
from types import SimpleNamespace

from onero_a1_bringup.hardware_mode import (
    MODE_DEFAULTS, configure_hardware_mode, resolve_hardware_settings,
)


class HardwareModeTest(unittest.TestCase):
    def test_default_and_explicit_modes(self):
        self.assertEqual(resolve_hardware_settings()["hardware_mode"], "sim")
        self.assertEqual(set(MODE_DEFAULTS), {"sim", "real"})
        for mode, simulated in (("sim", "true"), ("real", "false")):
            result = resolve_hardware_settings(mode)
            self.assertEqual(result, {"hardware_mode": mode, "onero_simulate": simulated})

    def test_ambiguous_mode_switches_fail(self):
        for name in ("use_fake_hardware", "simulate"):
            for value in ("true", "false"):
                context = SimpleNamespace(launch_configurations={name: value})
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    configure_hardware_mode(context)

    def test_invalid_selections_fail(self):
        for mode in ("fake", "typo", "", "false"):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                resolve_hardware_settings(mode)


if __name__ == "__main__":
    unittest.main()
