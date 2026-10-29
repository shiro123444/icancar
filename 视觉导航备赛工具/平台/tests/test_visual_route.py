import unittest
from automation.visual_route import RouteGapPlanner


class RouteGapTests(unittest.TestCase):
    def obs(self,boxes):return dict(valid=True,confidence=1.,lane=[.1,.9],blue_boxes=[{'box':b} for b in boxes],
       steering_hint=.1,left_margin=.5,right_margin=.5,heading_error=0.)
    def test_second_barrier_requires_left_and_does_not_take_the_nearer_right_gap(self):
        p=RouteGapPlanner(min_gap_ratio=.4)
        first=p.propose(self.obs([[.1,.3,.45,.7]]),1.,0.)
        self.assertEqual(first['required_side'],'right');self.assertLess(first['steering'],0)
        p.propose(self.obs([]),1.7,0.)
        second=p.propose(self.obs([[.55,.3,.9,.7]]),2.,0.)
        self.assertEqual(second['required_side'],'left');self.assertGreater(second['steering'],0)
        wrong=p.propose(self.obs([[.1,.3,.6,.7]]),2.1,0.)
        self.assertEqual(wrong['mode'],'route_gap_unresolved')
        self.assertIsNone(wrong['steering'])
    def test_frames_do_not_increment_encounter_count(self):
        p=RouteGapPlanner()
        for i in range(10):p.propose(self.obs([[.1,.3,.4,.7]]),i*.01,0.)
        self.assertEqual(p.encounters,1)
    def test_barrier_sequence_repeats_on_the_next_lap(self):
        p=RouteGapPlanner()
        self.assertEqual(p.propose(self.obs([[.1,.3,.4,.7]]),1.,0.)['required_side'],'right')
        p.propose(self.obs([]),1.6,0.)
        self.assertEqual(p.propose(self.obs([[.6,.3,.9,.7]]),2.,0.)['required_side'],'left')
        p.propose(self.obs([]),2.6,0.)
        self.assertEqual(p.propose(self.obs([[.1,.3,.4,.7]]),10.,0.)['required_side'],'right')
    def test_missing_geometry_is_not_claimed_as_a_valid_gap(self):
        r=RouteGapPlanner().propose({'valid':False},0.,.1)
        self.assertEqual(r['mode'],'model_only');self.assertTrue(r['review_required'])
    def test_blue_background_is_not_a_barrier_when_the_road_is_clear(self):
        try:import cv2
        except ImportError:self.skipTest('OpenCV available in calibration environment')
        import numpy as np
        from automation.visual_route import observe_lane
        image=np.zeros((240,320,3),np.uint8);image[:]=[0,80,180]
        polygon=np.array([[130,80],[190,80],[315,239],[5,239]],np.int32)
        cv2.fillPoly(image,[polygon],(130,130,130))
        cv2.line(image,(130,80),(5,239),(240,220,0),8)
        cv2.line(image,(190,80),(315,239),(240,220,0),8)
        value=observe_lane(image)
        self.assertTrue(value['valid']);self.assertEqual(value['blue_boxes'],[])
    def test_blue_panel_inside_the_lane_remains_a_candidate(self):
        try:import cv2
        except ImportError:self.skipTest('OpenCV available in calibration environment')
        import numpy as np
        from automation.visual_route import observe_lane
        image=np.full((240,320,3),130,np.uint8)
        cv2.line(image,(80,80),(20,239),(240,220,0),8)
        cv2.line(image,(240,80),(300,239),(240,220,0),8)
        cv2.rectangle(image,(85,105),(140,205),(0,80,180),-1)
        value=observe_lane(image)
        self.assertTrue(value['valid']);self.assertEqual(len(value['blue_boxes']),1)
        self.assertLess(value['blue_boxes'][0]['box'][2],.5)
