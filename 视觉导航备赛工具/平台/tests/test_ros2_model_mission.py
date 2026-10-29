"""Exercise the mission adapter with fake ROS messages, never real hardware."""
import io
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image

from ros2 import automatic_race
from test_ros2_control_adapters import FakeNode, Message, Odometry, Stamp, fake_modules


class CompressedImage:
    def __init__(self, stamp, line=False):
        self.header = types.SimpleNamespace(stamp=Stamp(stamp))
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        if line:
            image[80:86, :, :] = 255
        out = io.BytesIO()
        Image.fromarray(image).save(out, format='JPEG')
        self.data = out.getvalue()


class ModelMissionAdapterTests(unittest.TestCase):
    def test_vendor_model_drives_stop_and_loss_faults(self):
        clock = [1.0]
        def step(node, index, *, line=False, crossing=False):
            clock[0] = 1 + index / 10
            FakeNode.now_ns = 10_000_000_000 + index * 100_000_000
            node.camera(CompressedImage(FakeNode.now_ns, line))
            odom = Odometry()
            odom.header.stamp = Stamp(FakeNode.now_ns)
            node.odom(odom)
            boxes = [{'label': 'crossing', 'score': .9, 'box': [20, 30, 80, 80]}] if crossing else []
            payload = {'source':'openvino_race', 'valid':True, 'stamp_ns':FakeNode.now_ns,
                       'width':100, 'height':100, 'inference_ms':10, 'detections':boxes}
            node.model(Message(json.dumps(payload)))
            return payload

        def spin(node):
            self.assertTrue(all(timer[2].clock_type == 3 for timer in node.timers))
            for i in range(3):
                step(node, i)
            response = node.arm(None, types.SimpleNamespace(success=False, message=''))
            self.assertTrue(response.success, response.message)
            for i in range(3, 6):
                step(node, i, line=True)
            self.assertEqual(node.race.phase, 'running')
            first = step(node, 6, crossing=True)
            node.model(Message(json.dumps(first)))  # same result is not a second observation
            self.assertEqual(node.race.phase, 'running')
            step(node, 7, crossing=True)
            self.assertEqual(node.race.phase, 'running')
            step(node, 8, crossing=True)
            self.assertEqual((node.race.phase, node.race.status(clock[0])['action']), ('yield', 'stop'))
            node.model(Message('{"source":"openvino_race","valid":false}'))
            self.assertEqual((node.race.phase, node.race.reason), ('fault', 'crosswalk_model_unavailable'))

        config = {'calibrated':True,
                  'start_line':{'roi':[0,.7,1,.95], 'axis':'y', 'white_threshold':.75,
                                'min_coverage':.6, 'min_band_fraction':.05, 'min_bands':1, 'max_bands':1},
                  'crosswalk':{'roi':[0,.3,1,.7], 'axis':'x', 'white_threshold':.75,
                               'min_coverage':.4, 'min_band_fraction':.02, 'min_bands':3, 'max_bands':12}}
        model_config = {'calibrated':True, 'min_confidence':.5, 'roi':[0,0,1,1],
                        'max_source_age_s':.8, 'max_gap_s':1}
        with tempfile.TemporaryDirectory() as directory:
            config_path, model_path = Path(directory)/'stripe.json', Path(directory)/'model.json'
            config_path.write_text(json.dumps(config), encoding='utf-8')
            model_path.write_text(json.dumps(model_config), encoding='utf-8')
            modules = fake_modules(spin)
            sensor = types.ModuleType('sensor_msgs.msg')
            sensor.Image, sensor.CompressedImage = type('Image', (), {}), CompressedImage
            modules['sensor_msgs'], modules['sensor_msgs.msg'] = types.ModuleType('sensor_msgs'), sensor
            args = ['automatic_race.py', '--camera-topic','/trial/image', '--camera-type','compressed',
                    '--odom-topic','/trial/odom', '--config',str(config_path),
                    '--crosswalk-source','model', '--model-config',str(model_path)]
            with patch.dict(sys.modules, modules), patch.object(sys, 'argv', args), \
                    patch.object(automatic_race.time, 'monotonic', side_effect=lambda: clock[0]):
                automatic_race.main()


if __name__ == '__main__':
    unittest.main()
