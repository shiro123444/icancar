import unittest

from automation.model_crosswalk import ModelCrosswalk, validate_model_config, geometry_settings, verify_crosswalk_box
from automation.race import Race


CONFIG = {'calibrated': False, 'min_confidence': .5, 'roi': [.1, .3, .9, .9],
          'max_source_age_s': .8, 'max_gap_s': 1.0}


def result(stamp, detections=None):
    return {'source': 'openvino_race', 'valid': True, 'stamp_ns': stamp,
            'width': 640, 'height': 480, 'inference_ms': 12,
            'detections': [] if detections is None else detections}


class ModelCrosswalkTests(unittest.TestCase):
    def test_bound_mapping_and_same_frame_geometry_are_required(self):
        c=dict(CONFIG,class_order=['start','crossing'],model_sha256='a'*64,require_stripes=True)
        guard=ModelCrosswalk(c)
        item={'label':'crossing','class_id':1,'score':.8,'box':[200,140,400,350],
              'stripe_verified':False,'stripe_bands':1}
        value=result(10_000_000_000,[item])
        value.update(class_order=c['class_order'],model_sha256=c['model_sha256'],stripe_geometry=geometry_settings(c))
        self.assertIs(guard.update(value,1,10_000_000_000),False)
        item.update(stripe_verified=True,stripe_bands=4)
        value['stamp_ns']=10_100_000_000
        self.assertIs(guard.update(value,1.1,10_100_000_000),True)
        value['stamp_ns']=10_200_000_000
        value['class_order']=['crossing','start']
        self.assertIsNone(guard.update(value,1.2,10_200_000_000))
        self.assertFalse(guard.available(1.2,10_200_000_000))

    def test_real_start_stripe_and_crosswalks_have_different_structure(self):
        from pathlib import Path
        import numpy as np
        from PIL import Image
        folder=Path(__file__).resolve().parents[1]/'data/现场采集/20261029-充电后联调'
        cases=[('起点-1139.jpg',[0,424.7,569.2,480],False),
               ('人行道前30cm.jpg',[271.8,271.3,640,386.9],True),
               ('人行道前50cm.jpg',[101.4,239.9,461.5,291.5],True)]
        if not all((folder/name).is_file() for name,_,_ in cases):self.skipTest('site reference images unavailable')
        for name,box,expected in cases:
            image=np.asarray(Image.open(folder/name).convert('RGB'))
            self.assertIs(verify_crosswalk_box(image,box,CONFIG)['stripe_verified'],expected,name)

    def test_confidence_region_and_freshness(self):
        guard = ModelCrosswalk(dict(CONFIG))
        crossing = {'label': 'crossing', 'score': .7, 'box': [200, 140, 400, 350]}
        self.assertIs(guard.update(result(10_000_000_000, [crossing]), 1, 10_100_000_000), True)
        self.assertTrue(guard.available(1.2, 10_500_000_000))
        self.assertFalse(guard.available(1.3, 10_810_000_000))
        crossing['score'] = .4
        self.assertIs(guard.update(result(11_000_000_000, [crossing]), 2, 11_000_000_000), False)
        crossing['score'], crossing['box'] = .8, [0, 0, 30, 40]
        self.assertIs(guard.update(result(11_100_000_000, [crossing]), 2.1, 11_100_000_000), False)

    def test_bad_invalid_or_future_result_cannot_be_treated_as_clear(self):
        guard = ModelCrosswalk(dict(CONFIG))
        self.assertIs(guard.update(result(10_000_000_000), 1, 10_000_000_000), False)
        self.assertIsNone(guard.update({'source': 'openvino_race', 'valid': False}, 1.1, 10_100_000_000))
        self.assertFalse(guard.available(1.1, 10_100_000_000))
        bad = result(10_200_000_000, [{'label': 'crossing', 'score': .8, 'box': [10, 10, 700, 20]}])
        self.assertIsNone(guard.update(bad, 1.2, 10_200_000_000))
        self.assertFalse(guard.available(1.2, 10_200_000_000))
        self.assertIsNone(guard.update(result(30_000_000_000), 1.3, 10_300_000_000))
        self.assertIs(guard.update(result(10_400_000_000), 1.4, 10_400_000_000), False)

    def test_unique_model_observations_drive_stop_and_duplicates_do_not(self):
        guard = ModelCrosswalk(dict(CONFIG))
        race = Race(frames=3)
        for i in range(3):
            t = 1 + i * .1
            stamp = 10_000_000_000 + i * 100_000_000
            race.frame(t, False, None)
            race.odom(t, 0, 0, 0)
            obs = guard.update(result(stamp), t, stamp)
            race.crosswalk_frame(t, obs)
        race.arm(1.2)
        for i in range(3):
            t = 1.3 + i * .1
            race.frame(t, True, None)
            race.odom(t, 0, 0, 0)
        self.assertEqual(race.phase, 'running')
        hit = {'label': 'crossing', 'score': .8, 'box': [200, 140, 400, 350]}
        stamp = 10_600_000_000
        self.assertIs(guard.update(result(stamp, [hit]), 1.6, stamp), True)
        race.crosswalk_frame(1.6, True)
        for i in range(2):
            t = 1.7 + .1 * i
            race.frame(t, False, None)
            race.odom(t, 0, 0, 0)
            self.assertIsNone(guard.update(result(stamp, [hit]), t, stamp + (i + 1) * 100_000_000))
        self.assertEqual(race.phase, 'running')
        for i in range(2):
            t = 1.9 + .1 * i
            stamp = 10_900_000_000 + i * 100_000_000
            race.frame(t, False, None)
            race.odom(t, 0, 0, 0)
            obs = guard.update(result(stamp, [hit]), t, stamp)
            race.crosswalk_frame(t, obs)
        self.assertEqual((race.phase, race.status(2)['action']), ('yield', 'stop'))

    def test_missing_model_faults_running_mission(self):
        guard = ModelCrosswalk(dict(CONFIG))
        race = Race(frames=1)
        race.frame(1, False, False)
        race.odom(1, 0, 0, 0)
        race.arm(1)
        race.frame(1.1, True, None)
        self.assertEqual(race.phase, 'running')
        self.assertFalse(guard.available(1.1, 10_100_000_000))
        race.fail(1.1, 'crosswalk_model_unavailable')
        self.assertEqual((race.phase, race.status(1.1)['action']), ('fault', 'stop'))

    def test_configuration_requires_real_roi_and_timeouts(self):
        with self.assertRaises(ValueError):
            validate_model_config(dict(CONFIG, roi=[0, 0, 0, 1]))
        with self.assertRaises(ValueError):
            validate_model_config(dict(CONFIG, max_gap_s=float('nan')))
        with self.assertRaises(ValueError):
            validate_model_config(dict(CONFIG,class_order=[[],[]]))
        with self.assertRaises(ValueError):
            validate_model_config(dict(CONFIG,require_stripes=True,stripe_geometry=None))


if __name__ == '__main__':
    unittest.main()
