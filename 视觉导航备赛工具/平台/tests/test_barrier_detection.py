import importlib.util
import json
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipIf(cv2 is None or np is None, 'OpenCV and NumPy are required')
class BarrierDetectionTests(unittest.TestCase):
    def setUp(self):
        from automation.barrier_detection import detect_barriers
        self.detect = detect_barriers
        self.config = json.loads((Path(__file__).resolve().parents[1] / 'data/barrier.example.json').read_text())
        self.config['calibrated'] = True

    @staticmethod
    def scene(board=False, lane=True):
        image = np.full((240, 320, 3), 130, dtype=np.uint8)
        image[:, :60] = (0, 60, 220)
        image[:, 260:] = (0, 60, 220)
        if lane:
            image[:, 60:68] = (250, 220, 0)
            image[:, 252:260] = (250, 220, 0)
        if board:
            image[70:180, 100:145] = (0, 55, 210)
        return image

    def test_blue_floor_outside_yellow_lane_is_not_a_barrier(self):
        result = self.detect(self.scene(), self.config)
        self.assertTrue(result['valid'])
        self.assertEqual(result['boxes'], [])
        self.assertLess(result['lane'][0], .3)
        self.assertGreater(result['lane'][1], .7)

    def test_blue_panel_inside_lane_is_detected(self):
        result = self.detect(self.scene(board=True), self.config)
        self.assertTrue(result['valid'])
        self.assertEqual(len(result['boxes']), 1)
        x1, y1, x2, y2 = result['boxes'][0]['box']
        self.assertLess(x1, .33)
        self.assertGreater(x2, .44)
        self.assertGreater(y2, .7)

    def test_missing_boundaries_or_uncalibrated_config_cannot_clear(self):
        result = self.detect(self.scene(board=True, lane=False), self.config)
        self.assertFalse(result['valid'])
        self.config['calibrated'] = False
        result = self.detect(self.scene(board=True), self.config)
        self.assertFalse(result['valid'])
        self.assertEqual(len(result['boxes']), 1)  # candidates still visible for calibration


if __name__ == '__main__':
    unittest.main()
