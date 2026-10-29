import json
import unittest
from pathlib import Path

from automation.barrier_avoidance import BarrierAvoider, ZERO


def observation(stamp, boxes=(), valid=True):
    return {'source':'barrier_detector', 'valid':valid, 'calibrated':True, 'stamp_ns':stamp,
            'lane':[.1,.9], 'boxes':[{'box':box} for box in boxes]}


class BarrierAvoidanceTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((Path(__file__).resolve().parents[1]/'data/avoidance.example.json').read_text())
        self.config['calibrated'] = True
        self.planner = BarrierAvoider(self.config)
        self.stamp = 10_000_000_000
        self.planner.set_lane(.05, 0, .1, self.stamp, 1, self.stamp)

    def test_clear_lane_follows_and_panel_steers_toward_gap(self):
        self.assertTrue(self.planner.set_observation(observation(self.stamp), 1, self.stamp))
        self.assertEqual(self.planner.output(1.01,self.stamp+10_000_000)[2], 'follow_lane')
        stamp=self.stamp+100_000_000
        self.planner.set_observation(observation(stamp, [[.1,.2,.55,.7]]),1.1,stamp)
        command, _, reason=self.planner.output(1.1,stamp)
        self.assertEqual(reason,'avoid_barrier')
        self.assertLessEqual(command[0],.04)
        self.assertLess(command[2],0)  # image right requires negative yaw under this example sign

    def test_no_gap_or_close_panel_stops(self):
        self.planner.set_observation(observation(self.stamp, [[.1,.2,.9,.7]]),1,self.stamp)
        self.assertEqual(self.planner.output(1,self.stamp), (ZERO,self.stamp,'no_safe_gap'))
        stamp=self.stamp+100_000_000
        self.planner.set_observation(observation(stamp, [[.1,.2,.45,.9]]),1.1,stamp)
        self.assertEqual(self.planner.output(1.1,stamp), (ZERO,self.stamp,'barrier_too_close'))

    def test_unavailable_or_uncalibrated_never_outputs_motion(self):
        self.assertEqual(self.planner.output(1,self.stamp)[0],ZERO)
        self.planner.set_observation(observation(self.stamp),1,self.stamp)
        self.assertEqual(self.planner.output(1.41,self.stamp+410_000_000)[0],ZERO)
        self.config['calibrated']=False
        other=BarrierAvoider(self.config)
        self.assertEqual(other.output(1,self.stamp)[2],'uncalibrated')

    def test_clear_hysteresis_and_replay_do_not_release_early(self):
        self.planner.set_observation(observation(self.stamp, [[.1,.2,.55,.7]]),1,self.stamp)
        self.assertEqual(self.planner.output(1,self.stamp)[2],'avoid_barrier')
        stamp=self.stamp+100_000_000
        clear=observation(stamp)
        self.planner.set_observation(clear,1.1,stamp)
        self.assertEqual(self.planner.output(1.1,stamp)[2],'confirming_clear')
        self.assertFalse(self.planner.set_observation(clear,1.2,stamp+100_000_000))
        self.assertEqual(self.planner.output(1.2,stamp+100_000_000)[2],'confirming_clear')
        for i in (2,3):
            s=self.stamp+i*100_000_000
            self.planner.set_observation(observation(s),1+i*.1,s)
        self.assertEqual(self.planner.output(1.3,self.stamp+300_000_000)[2],'follow_lane')

    def test_stop_lane_and_bad_observation_cannot_move(self):
        self.planner.set_observation(observation(self.stamp),1,self.stamp)
        stamp=self.stamp+100_000_000
        self.planner.set_lane(0,0,0,stamp,1.1,stamp)
        self.assertEqual(self.planner.output(1.1,stamp)[0],ZERO)
        self.assertFalse(self.planner.set_observation(observation(stamp,valid=False),1.1,stamp))
        self.assertEqual(self.planner.output(1.1,stamp)[0],ZERO)

    def test_speed_limits_and_invalid_lane_geometry(self):
        self.planner.set_lane(.2,0,2,self.stamp+1,1,self.stamp+1)
        view=observation(self.stamp+1)
        self.planner.set_observation(view,1,self.stamp+1)
        command,_,reason=self.planner.output(1,self.stamp+1)
        self.assertEqual((command,reason),((.05,0,.35),'follow_lane'))
        view=observation(self.stamp+2)
        view['lane']=[.1,.4]
        self.planner.set_observation(view,1.01,self.stamp+2)
        self.assertEqual(self.planner.output(1.01,self.stamp+2)[2],'lane_geometry_invalid')


if __name__ == '__main__':
    unittest.main()
