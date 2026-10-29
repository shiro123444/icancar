"""Exercise the detector's ROS adapter with synthetic images and fake ROS."""
import json
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from test_ros2_control_adapters import FakeNode, Stamp, fake_modules


@unittest.skipIf(cv2 is None or np is None, 'OpenCV and NumPy are required')
class BarrierDetectorAdapterTests(unittest.TestCase):
    def test_synthetic_frame_emits_candidate_and_invalid_frame_revokes_it(self):
        from ros2 import barrier_detector
        class CompressedImage:
            def __init__(self, stamp, data):
                self.header=types.SimpleNamespace(stamp=Stamp(stamp))
                self.data=data
        def spin(node):
            image=np.full((240,320,3),130,dtype=np.uint8)
            image[:,:60]=(0,60,220);image[:,260:] = (0,60,220)
            image[:,60:68]=(250,220,0);image[:,252:260]=(250,220,0)
            image[70:170,75:150]=(0,55,210)
            ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            self.assertTrue(ok)
            FakeNode.now_ns=time.time_ns()
            node.frame(CompressedImage(FakeNode.now_ns,encoded.tobytes()))
            deadline=time.monotonic()+3
            while node.worker.stats()['processed']<1 and time.monotonic()<deadline:
                time.sleep(.01)
            node.publish_result()
            value=json.loads(node.pub.messages[-1].data)
            self.assertTrue(value['valid'])
            self.assertEqual(len(value['boxes']),1)
            FakeNode.now_ns+=100_000_000
            node.frame(CompressedImage(FakeNode.now_ns,b'bad-image'))
            self.assertFalse(json.loads(node.pub.messages[-1].data)['valid'])

        config=json.loads((Path(__file__).resolve().parents[1]/'data/barrier.example.json').read_text())
        config['calibrated']=True
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'barrier.json';path.write_text(json.dumps(config))
            modules=fake_modules(spin)
            sensor=types.ModuleType('sensor_msgs.msg');sensor.CompressedImage=CompressedImage
            modules['sensor_msgs'],modules['sensor_msgs.msg']=types.ModuleType('sensor_msgs'),sensor
            args=['barrier_detector.py','--config',str(path),'--max-source-age','3']
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',args):barrier_detector.main()


if __name__=='__main__':unittest.main()
