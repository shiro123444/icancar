import math
import unittest
from automation.steering_candidate import SteeringCandidate, fresh_frame


class SteeringCandidateTests(unittest.TestCase):
    def test_opt_in_limits_and_stale_stop(self):
        preview = {'source': 'steering_cnn', 'valid': True, 'steering': 1.7}
        disabled = SteeringCandidate(.05, .4, .5)
        disabled.update(preview, 10)
        self.assertIsNone(disabled.output(10.1))
        active = SteeringCandidate(.05, .4, .5, enabled=True)
        active.update(preview, 10)
        self.assertEqual(active.output(10.1), (.05, 0.0, .4))
        self.assertEqual(active.output(10.6), (0.0, 0.0, 0.0))
        active.update({'source': 'steering_cnn', 'valid': False}, 11)
        self.assertEqual(active.output(11.1), (0.0, 0.0, 0.0))

    def test_invalid_predictions_stop(self):
        active = SteeringCandidate(.05, .4, .5, sign=-1, enabled=True)
        active.update({'source': 'steering_cnn', 'valid': True, 'steering': -.2}, 10)
        self.assertEqual(active.output(10.1), (.05, 0.0, .2))
        for value in (float('nan'), float('inf'), True, None):
            active.update({'source': 'steering_cnn', 'valid': True, 'steering': value}, 11)
            self.assertEqual(active.output(11.1), (0.0, 0.0, 0.0))
        active.update({'source': 'other', 'valid': True, 'steering': .2}, 12)
        self.assertEqual(active.output(12.1), (0.0, 0.0, 0.0))

    def test_image_age_is_checked_at_output_not_just_message_arrival(self):
        candidate = SteeringCandidate(.05, .4, .5, enabled=True, max_frame_age=.4)
        prediction = {'source': 'steering_cnn', 'valid': True, 'steering': .2,
                      'image_stamp_ns': 10_000_000_000}
        candidate.update(prediction, 5.0)
        self.assertEqual(candidate.output(5.1, 10_200_000_000), (.05, 0.0, .2))
        # The message is still recent, but its source image has expired.
        self.assertEqual(candidate.output(5.2, 10_500_000_000), (0.0, 0.0, 0.0))
        # Repeated arrivals of an old prediction cannot keep the vehicle moving.
        candidate.update(prediction, 6.0)
        self.assertEqual(candidate.output(6.01, 11_000_000_000), (0.0, 0.0, 0.0))
        candidate.update({'source': 'steering_cnn', 'valid': True, 'steering': .2}, 7.0)
        self.assertEqual(candidate.output(7.01, 11_100_000_000), (0.0, 0.0, 0.0))
        self.assertFalse(fresh_frame(0, 1, .4))
        self.assertFalse(fresh_frame(10_000_000_000, 9_500_000_000, .4))
        self.assertFalse(fresh_frame(True, 10_000_000_000, .4))
        self.assertFalse(fresh_frame(10**400, 10_000_000_000, .4))


if __name__ == '__main__':
    unittest.main()
