import json
import unittest
from pathlib import Path
from automation.map_localization import MapLocalizer


class MapLocalizationTests(unittest.TestCase):
    def make(self):return MapLocalizer(json.loads((Path(__file__).resolve().parents[1]/'data/track.json').read_text()))
    def test_unanchored_is_not_presented_as_a_real_position(self):
        self.assertEqual(self.make().update([0,0,0],1)['state'],'waiting_for_anchor')
    def test_forward_and_positive_left_yaw_have_correct_map_signs(self):
        m=self.make();m.seed(1,[0,0,0],-90)
        a=m.update([0,0,0],1);b=m.update([1,.1,.2],3)
        self.assertLess(b['odometry']['y'],a['odometry']['y'])
        self.assertLess(b['odometry']['x'],a['odometry']['x'])
        self.assertLess(b['odometry']['heading_deg'],-90)
        self.assertFalse(b['metric_calibrated'])
    def test_reverse_heading_and_base_reset_are_visible(self):
        m=self.make();m.seed(1,[0,0,0],90)
        a=m.update([0,0,0],1)
        self.assertEqual(a['direction'],'reverse')
        m.update([1,0,0],3)
        self.assertEqual(m.update([0,0,0],3.1)['state'],'anchor_required')
    def test_read_only_runtime_requires_an_explicit_start_session_or_manual_seed(self):
        from automation.map_localization import LocalizationRuntime
        track=self.make().map.track;runtime=LocalizationRuntime(track,clock=lambda:1.)
        state={'control':{'session':'a','pose':[0,0,0],'origin':[0,0,0]},'control_age_s':.1}
        self.assertEqual(runtime.ingest(state)['state'],'waiting_for_anchor')
        state['control']['start_anchor']=1
        self.assertEqual(runtime.ingest(state)['state'],'tracking')
        self.assertFalse(runtime.ingest(state)['metric_calibrated'])
    def test_wrong_junction_branch_is_reported_instead_of_jumping_route_sections(self):
        m=self.make();m.seed(5,[0,0,0],90)
        m.update([0,0,0],1)
        forward=50*m.map.sy
        m.update([forward,0,0],3)
        value=m.update([forward,0,1.5707963267948966],4)
        self.assertTrue(value['branch_conflict'])
    def test_new_session_without_start_anchor_discards_old_alignment(self):
        from automation.map_localization import LocalizationRuntime
        runtime=LocalizationRuntime(self.make().map.track,clock=lambda:1.)
        old={'session':'a','pose':[0,0,0],'origin':[0,0,0],'start_anchor':1}
        self.assertEqual(runtime.ingest({'control':old,'control_age_s':.1})['state'],'tracking')
        new={'session':'b','pose':[0,0,0],'origin':[0,0,0]}
        self.assertEqual(runtime.ingest({'control':new,'control_age_s':.1})['state'],'waiting_for_anchor')
    def test_odometry_reset_does_not_reapply_an_old_session_start(self):
        from automation.map_localization import LocalizationRuntime
        now=[1.]
        runtime=LocalizationRuntime(self.make().map.track,clock=lambda:now[0])
        control={'session':'a','pose':[0,0,0],'origin':[0,0,0],'start_anchor':1}
        runtime.ingest({'control':control,'control_age_s':.1})
        now[0]=3.;control['pose']=[1,0,0]
        runtime.ingest({'control':control,'control_age_s':.1})
        now[0]=3.1;control['pose']=[0,0,0]
        self.assertEqual(runtime.ingest({'control':control,'control_age_s':.1})['state'],'anchor_required')
        now[0]=3.2
        self.assertEqual(runtime.ingest({'control':control,'control_age_s':.1})['state'],'waiting_for_anchor')
