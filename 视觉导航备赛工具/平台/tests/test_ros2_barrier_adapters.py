"""Connect avoidance and range adapters to simulated messages only."""
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from ros2 import barrier_avoidance, range_safety
from test_ros2_control_adapters import FakeNode, Message, Stamp, fake_modules, stamped


class RangeMessage:
    def __init__(self, stamp, distance):
        self.header = types.SimpleNamespace(stamp=Stamp(stamp))
        self.range, self.min_range, self.max_range = distance, .05, 2.0


class BarrierAdapterTests(unittest.TestCase):
    def test_avoidance_candidate_and_near_panel_stop(self):
        clock = [1.0]
        def spin(node):
            self.assertTrue(all(timer[2].clock_type == 3 for timer in node.timers))
            FakeNode.now_ns = 10_000_000_000
            node.lane(stamped(FakeNode.now_ns,.05,.1))
            obs = {'source':'barrier_detector','valid':True,'calibrated':True,'stamp_ns':FakeNode.now_ns,
                   'lane':[.1,.9],'boxes':[{'box':[.1,.2,.55,.7]}]}
            node.observation(Message(json.dumps(obs)))
            node.update()
            moving=node.publisher.messages[-1]
            self.assertGreater(moving.twist.linear.x,0)
            self.assertLess(moving.twist.angular.z,0)
            count=len(node.publisher.messages)
            clock[0],FakeNode.now_ns=1.1,10_100_000_000
            obs['stamp_ns']=FakeNode.now_ns
            obs['boxes']=[{'box':[.1,.2,.55,.9]}]
            node.observation(Message(json.dumps(obs)))
            self.assertEqual(len(node.publisher.messages),count+1)
            self.assertEqual(node.publisher.messages[-1].twist.linear.x,0)
            node.observation(Message('{"source":"barrier_detector","valid":false}'))
            self.assertEqual(node.publisher.messages[-1].twist.linear.x,0)

        config=json.loads((Path(__file__).resolve().parents[1]/'data/avoidance.example.json').read_text())
        config['calibrated']=True
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json';path.write_text(json.dumps(config))
            args=['barrier_avoidance.py','--config',str(path),'--activate']
            with patch.dict(sys.modules,fake_modules(spin)), patch.object(sys,'argv',args), \
                    patch.object(barrier_avoidance.time,'monotonic',side_effect=lambda:clock[0]):
                barrier_avoidance.main()

    def test_range_adapter_fails_closed_and_clears_only_after_three_samples(self):
        clock=[1.0]
        def spin(node):
            self.assertTrue(all(timer[2].clock_type == 3 for timer in node.timers))
            for i in range(3):
                clock[0]=1+i*.1;FakeNode.now_ns=10_000_000_000+i*100_000_000
                node.range(RangeMessage(FakeNode.now_ns,1.0))
            node.publish()
            self.assertTrue(node.publisher.messages[-1].data)
            count=len(node.publisher.messages)
            clock[0],FakeNode.now_ns=1.3,10_300_000_000
            node.range(RangeMessage(FakeNode.now_ns,.2))
            self.assertEqual(len(node.publisher.messages),count+1)
            self.assertFalse(node.publisher.messages[-1].data)
        config=json.loads((Path(__file__).resolve().parents[1]/'data/range-safety.example.json').read_text())
        config['calibrated']=True
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'range.json';path.write_text(json.dumps(config))
            modules=fake_modules(spin)
            sensor=types.ModuleType('sensor_msgs.msg');sensor.Range=RangeMessage
            modules['sensor_msgs'],modules['sensor_msgs.msg']=types.ModuleType('sensor_msgs'),sensor
            args=['range_safety.py','--config',str(path),'--range-topic','/trial/range']
            with patch.dict(sys.modules,modules),patch.object(sys,'argv',args), \
                    patch.object(range_safety.time,'monotonic',side_effect=lambda:clock[0]):
                range_safety.main()


if __name__ == '__main__':
    unittest.main()
