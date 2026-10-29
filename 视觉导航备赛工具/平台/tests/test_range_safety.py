import json
import unittest
from pathlib import Path

from automation.range_safety import RangeSafety


class RangeSafetyTests(unittest.TestCase):
    def setUp(self):
        config=json.loads((Path(__file__).resolve().parents[1]/'data/range-safety.example.json').read_text())
        config['calibrated']=True
        self.guard=RangeSafety(config)

    def push(self, distance, index, minimum=.05, maximum=2):
        stamp=10_000_000_000+index*100_000_000
        self.guard.update(distance,minimum,maximum,stamp,1+index*.1,stamp)
        return self.guard.allowed(1+index*.1,stamp)

    def test_requires_unique_clear_measurements_and_stops_immediately(self):
        self.assertFalse(self.push(1,0))
        self.assertFalse(self.push(1,1))
        self.assertTrue(self.push(1,2))
        self.assertFalse(self.push(.2,3))
        self.assertFalse(self.push(.5,4))
        self.assertFalse(self.push(.5,5))
        self.assertTrue(self.push(.5,6))

    def test_stale_invalid_and_unsupported_range_never_clear(self):
        for i in range(3):self.push(1,i)
        self.assertFalse(self.guard.allowed(1.8,10_800_000_000))
        self.assertFalse(self.push(float('nan'),9))
        self.assertFalse(self.push(float('inf'),10))
        self.assertFalse(self.push(1,11,minimum=.3))
        self.assertFalse(self.push(1,12,maximum=.35))

    def test_repeated_sample_cannot_reopen(self):
        self.assertFalse(self.push(1,0))
        for _ in range(5):
            self.assertFalse(self.guard.update(1,.05,2,10_000_000_000,1.1,10_100_000_000))
        self.assertFalse(self.guard.allowed(1.1,10_100_000_000))

    def test_replayed_close_measurement_still_blocks(self):
        for i in range(3):self.push(1,i)
        self.assertFalse(self.guard.update(.2,.05,2,10_100_000_000,1.21,10_210_000_000))
        self.assertFalse(self.guard.allowed(1.21,10_210_000_000))

    def test_uncalibrated_is_always_blocked(self):
        self.guard.config['calibrated']=False
        for i in range(4):self.assertFalse(self.push(1,i))


if __name__ == '__main__':
    unittest.main()
