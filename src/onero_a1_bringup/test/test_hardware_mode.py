"""Mode selection must resolve before any hardware startup."""
import unittest

from onero_a1_bringup.hardware_mode import resolve_hardware_settings


class HardwareModeTest(unittest.TestCase):
    def test_default_and_explicit_modes(self):
        self.assertEqual(resolve_hardware_settings()["hardware_mode"], "fake")
        for mode, flags in (("fake", ("true", "true")),
                            ("sim", ("false", "true")),
                            ("real", ("false", "false"))):
            result = resolve_hardware_settings(mode)
            self.assertEqual((result["use_fake_hardware"], result["simulate"]), flags)
            self.assertEqual(resolve_hardware_settings(mode, *flags), result)

    def test_legacy_commands(self):
        self.assertEqual(resolve_hardware_settings(use_fake_hardware="false")["hardware_mode"], "sim")
        self.assertEqual(resolve_hardware_settings(None, "false", "false")["hardware_mode"], "real")
        # simulate alone never selected real hardware in the legacy interface.
        self.assertEqual(resolve_hardware_settings(simulate="false")["hardware_mode"], "fake")

    def test_conflicting_or_invalid_selections_fail(self):
        for args in (("fake", "false", "false"), ("sim", None, "false"),
                     ("real", "true", None), ("typo", None, None),
                     (None, "typo", None)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                resolve_hardware_settings(*args)


if __name__ == "__main__":
    unittest.main()
