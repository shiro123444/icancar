import math
import unittest

from automation.command import VelocityBuffer


class VelocityBufferTests(unittest.TestCase):
    def test_limits_and_source_age_are_checked_again_at_use(self):
        buffer = VelocityBuffer(.1, .4, timeout=.5, max_source_age=.3)
        self.assertTrue(buffer.update(3, 4, 9, 1, 10_000_000_000, 10_050_000_000))
        vx, vy, w = buffer.output(1.1, 10_200_000_000)
        self.assertAlmostEqual(math.hypot(vx, vy), .1)
        self.assertEqual(w, .4)
        # The receive watchdog has not elapsed, but the gate message has aged out.
        self.assertEqual(buffer.output(1.2, 10_310_000_000), (0, 0, 0))

    def test_duplicate_or_reordered_motion_never_refreshes_watchdog(self):
        buffer = VelocityBuffer(.1, .4, timeout=.3, max_source_age=.5)
        self.assertTrue(buffer.update(.05, 0, .2, 1, 10_000_000_000, 10_000_000_000))
        self.assertFalse(buffer.update(.1, 0, -.2, 1.2, 10_000_000_000, 10_200_000_000))
        self.assertFalse(buffer.update(.1, 0, -.2, 1.21, 9_990_000_000, 10_210_000_000))
        self.assertEqual(buffer.output(1.22, 10_220_000_000), (.05, 0, .2))
        self.assertEqual(buffer.output(1.31, 10_310_000_000), (0, 0, 0))
        self.assertFalse(buffer.update(.1, 0, .2, 1.32, 10_000_000_000, 10_320_000_000))
        self.assertEqual(buffer.output(1.32, 10_320_000_000), (0, 0, 0))

    def test_bad_or_future_input_stops_without_poisoning_newer_commands(self):
        buffer = VelocityBuffer(.1, .4, timeout=.3, max_source_age=.3)
        self.assertTrue(buffer.update(.05, 0, 0, 1, 10_000_000_000, 10_000_000_000))
        self.assertFalse(buffer.update(.05, 0, 0, 1.1, 20_000_000_000, 10_100_000_000))
        self.assertEqual(buffer.output(1.1, 10_100_000_000), (0, 0, 0))
        self.assertTrue(buffer.update(.05, 0, 0, 1.2, 10_200_000_000, 10_200_000_000))
        self.assertFalse(buffer.update(float('nan'), 0, 0, 1.21, 10_210_000_000, 10_210_000_000))
        self.assertEqual(buffer.output(1.21, 10_210_000_000), (0, 0, 0))
        self.assertFalse(buffer.update(.05, 0, 0, 1.22, None, 10_220_000_000))
        self.assertEqual(buffer.output(1.22, 10_220_000_000), (0, 0, 0))

    def test_replayed_stop_clears_current_motion(self):
        buffer = VelocityBuffer(.1, .4, timeout=.3, max_source_age=.3)
        self.assertTrue(buffer.update(.05, 0, .2, 1, 10_000_000_000, 10_000_000_000))
        self.assertFalse(buffer.update(0, 0, 0, 1.1, 10_000_000_000, 10_100_000_000))
        self.assertEqual(buffer.output(1.1, 10_100_000_000), (0, 0, 0))
        self.assertTrue(buffer.update(.05, 0, .2, 1.2, 10_200_000_000, 10_200_000_000))
        self.assertEqual(buffer.output(1.2, 10_200_000_000), (.05, 0, .2))

    def test_clock_rollback_and_receive_timeout_stop(self):
        buffer = VelocityBuffer(.1, .4, timeout=.3, max_source_age=.5)
        self.assertTrue(buffer.update(.05, 0, 0, 1, 10_000_000_000, 10_000_000_000))
        self.assertEqual(buffer.output(1.1, 9_900_000_000), (0, 0, 0))
        self.assertTrue(buffer.update(.05, 0, 0, 2, 11_000_000_000, 11_000_000_000))
        self.assertEqual(buffer.output(2.31, 11_310_000_000), (0, 0, 0))

    def test_unstamped_compatibility_still_has_local_timeout(self):
        buffer = VelocityBuffer(.1, .4, timeout=.3, stamped=False)
        self.assertTrue(buffer.update(.05, 0, .2, 1))
        self.assertEqual(buffer.output(1.1), (.05, 0, .2))
        self.assertEqual(buffer.output(1.31), (0, 0, 0))

    def test_overflowing_finite_components_fail_closed(self):
        buffer = VelocityBuffer(.1, .4, timeout=.3)
        self.assertTrue(buffer.update(.05, 0, 0, 1, 10_000_000_000, 10_000_000_000))
        self.assertFalse(buffer.update(1.7e308, 1.7e308, 0, 1.1, 10_100_000_000, 10_100_000_000))
        self.assertEqual(buffer.output(1.1, 10_100_000_000), (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
