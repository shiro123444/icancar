import unittest
from automation.supervised_follow import FollowTrial
from tests.test_model_crosswalk import CONFIG,result


class SupervisedFollowTests(unittest.TestCase):
    def feed(self,t,now,line=False,crosswalk=False,v=0.,blue=0.,pose=(0.,0.,0.),visible=True):
        stamp=10_000_000_000+int(round(now*1e9))
        t.steering({'source':'steering_cnn','valid':True,'steering':.1,'image_stamp_ns':stamp},now,stamp)
        t.observation(result(stamp),now,stamp)
        t.odometry(now,stamp,stamp,v,0.,0.,*pose)
        t.image(now,stamp,stamp,line,crosswalk,blue,visible)
        return t.tick(now,stamp)
    def ready(self,**kwargs):
        t=FollowTrial(dict(CONFIG),**kwargs)
        for i in range(9):self.feed(t,i*.1)
        self.assertEqual(t.phase,'follow')
        return t
    def test_real_inputs_required_and_stale_fault_latches(self):
        t=FollowTrial(dict(CONFIG))
        self.assertEqual(t.tick(1,11_000_000_000),(0.,0.,0.))
        self.assertEqual(t.phase,'waiting')
        t=self.ready()
        command=self.feed(t,1.,v=.1)
        self.assertGreater(command[0],0)
        self.assertLessEqual(command[0],.2)
        self.assertEqual(t.tick(2.,12_000_000_000),(0.,0.,0.))
        self.assertEqual(t.phase,'fault')
        self.feed(t,2.1)
        self.assertEqual(t.phase,'fault')
    def test_yield_requires_three_seconds_of_actual_stationary_feedback_and_ack(self):
        t=self.ready(stop_after_yield=True)
        for i in range(3):self.feed(t,1.+i*.1,crosswalk=True,v=.1)
        self.assertEqual(t.phase,'yield')
        t.ack(t.announcement_id,True,1.3)
        for i in range(10):self.feed(t,1.3+i*.1,crosswalk=True,v=.1)
        self.assertIsNone(t.stationary_since)
        for i in range(29):self.feed(t,2.3+i*.1,crosswalk=True)
        self.assertEqual(t.phase,'yield')
        for i in range(5):self.feed(t,5.2+i*.1,crosswalk=True)
        self.assertEqual(t.phase,'done')
        self.assertEqual(t.stops,1)
        self.assertEqual(t.command,(0.,0.,0.))
    def test_one_lap_needs_travel_crosswalk_and_return_to_reference_area(self):
        t=self.ready();t.stops=1;t.distance=9
        for i in range(3):self.feed(t,1.+i*.1,line=True,pose=(2.,0.,0.))
        self.assertEqual(t.laps,0)
        for i in range(3):self.feed(t,1.3+i*.1,line=False)
        for i in range(3):self.feed(t,1.6+i*.1,line=True,pose=(.1,.1,.1))
        self.assertEqual((t.phase,t.laps),('done',1))
    def test_blue_in_path_and_feedback_overspeed_stop(self):
        t=self.ready()
        for i in range(3):self.feed(t,1.+i*.1,blue=.8)
        self.assertEqual(t.phase,'fault')
        self.assertEqual(t.command,(0.,0.,0.))
        t=self.ready();self.feed(t,1.,v=.4)
        self.assertEqual(t.reason,'wheel_feedback_limit')
        self.assertEqual(t.events[-2]['type'],'wheel_feedback_limit_sample')
        self.assertEqual(t.events[-2]['measured_velocity'][0],.4)
    def test_fresh_but_obscured_camera_view_stops(self):
        t=self.ready()
        for i in range(3):
            self.feed(t,1.+i*.1,visible=False)
        self.assertEqual(t.reason,'camera_view_obscured')
        self.assertEqual(t.command,(0.,0.,0.))
    def test_view_quality_distinguishes_flat_occlusion_from_lane_colour(self):
        try:import cv2
        except ImportError:self.skipTest('OpenCV available in calibration environment')
        import numpy as np
        from automation.supervised_follow import view_quality
        image=np.full((120,160,3),128,np.uint8)
        self.assertFalse(view_quality(image)['visible'])
        image[:,20:30]=[240,220,0]
        self.assertTrue(view_quality(image)['visible'])
    def test_trial_diagnostics_are_visible_without_claiming_calibration(self):
        import json
        from ros2.telemetry_bridge import parse_control
        t=self.ready();status=t.status(1.)
        value=parse_control(json.dumps(status))
        self.assertEqual(value['source'],'supervised_visual_test')
        self.assertFalse(value['calibrated'])
        status['calibrated']=True
        with self.assertRaises(ValueError):parse_control(json.dumps(status))
    def test_short_input_gap_holds_zero_and_only_new_real_data_resumes(self):
        t=self.ready();self.feed(t,1.,v=.1)
        self.assertEqual(t.tick(1.5,11_500_000_000),(0.,0.,0.))
        self.assertEqual(t.phase,'follow')
        self.assertIsNotNone(t.input_hold_at)
        self.assertGreater(self.feed(t,1.6,v=0.)[0],0)
        self.assertIsNone(t.input_hold_at)


if __name__=='__main__':unittest.main()
