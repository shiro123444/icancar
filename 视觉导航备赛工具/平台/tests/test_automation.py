import json
import math
import unittest
from pathlib import Path

from automation.race import Race
from automation.command import CommandGate
from automation.demo import demo
from automation.voice_protocol import VoiceBox, firmware_reply, HANDSHAKE, STOP_PROMPT
from automation.base_protocol import FrameDecoder, measured_sensors, measured_velocity, velocity_frame
from ros2.camera_node import read_calibration
from tools.ros2_camera_calibrate import signature, novel
import struct


class Driver:
    def __init__(self):
        self.race, self.t = Race(frames=2), 0.0
    def step(self, line=False, crosswalk=False, velocity=0.0):
        self.t += 0.1
        self.race.frame(self.t, line, crosswalk)
        self.race.odom(self.t, velocity, 0, 0)
        return self.race.tick(self.t)
    def repeat(self, n, **kwargs):
        for _ in range(n):
            result = self.step(**kwargs)
        return result
    def arm_and_start(self):
        self.repeat(2)
        self.race.arm(self.t)
        self.repeat(2, line=True)


class RaceTests(unittest.TestCase):
    def test_demo_runs_full_two_lap_sequence(self):
        result = demo()
        self.assertEqual(result['final']['phase'], 'finished')
        self.assertEqual(result['final']['laps'], 2)
        self.assertEqual(result['final']['crosswalk_stops'], 2)
        self.assertEqual(sum(e['type'] == 'announcement_complete' for e in result['timeline']), 2)
    def test_two_laps_require_clear_and_reentry(self):
        d = Driver()
        d.arm_and_start()
        self.assertEqual(d.race.phase, 'running')
        d.repeat(20, line=True)
        self.assertEqual(d.race.laps, 0)
        d.repeat(2)
        d.repeat(2, line=True)
        self.assertEqual(d.race.laps, 1)
        d.repeat(10, line=True)
        self.assertEqual(d.race.laps, 1)
        d.repeat(2)
        state = d.repeat(2, line=True)
        self.assertEqual((state['phase'], state['laps'], state['action']), ('finished', 2, 'stop'))
        elapsed = state['elapsed_seconds']
        self.assertEqual(d.repeat(4)['elapsed_seconds'], elapsed)

    def test_crosswalk_requires_both_continuous_stop_and_matching_ack(self):
        d = Driver()
        d.arm_and_start()
        state = d.repeat(2, crosswalk=True)
        self.assertEqual((state['phase'], state['action']), ('yield', 'stop'))
        self.assertEqual(len([e for e in d.race.pop_events() if e['type'] == 'announce']), 1)
        d.repeat(35, crosswalk=True)
        self.assertEqual(d.race.phase, 'yield')
        d.race.acknowledge(d.t, 999, True)
        self.assertEqual(d.race.phase, 'yield')
        d.race.acknowledge(d.t, 1, True)
        self.assertEqual(d.race.phase, 'running')
        self.assertEqual(d.race.yields, 1)
        d.repeat(5, crosswalk=True)
        self.assertEqual(d.race.yields, 1)
        d.repeat(2)
        d.repeat(2, crosswalk=True)
        self.assertEqual(d.race.phase, 'yield')

    def test_movement_restarts_the_dwell_and_false_ack_does_not_release(self):
        d = Driver()
        d.arm_and_start()
        d.repeat(2, crosswalk=True)
        d.race.acknowledge(d.t, 1, False)
        d.repeat(20, crosswalk=True)
        d.step(crosswalk=True, velocity=0.2)
        d.race.acknowledge(d.t, 1, True)
        d.repeat(29, crosswalk=True)
        self.assertEqual(d.race.phase, 'yield')
        d.repeat(3, crosswalk=True)
        self.assertEqual(d.race.phase, 'running')

    def test_missing_camera_or_odom_latches_fault(self):
        for missing in ('camera', 'odom'):
            d = Driver()
            d.arm_and_start()
            d.t += 0.7
            if missing == 'camera':
                state = d.race.odom(d.t, 0, 0, 0)
                self.assertEqual(state['reason'], 'camera_timeout')
            else:
                # Keep the camera fresh without letting odometry refresh.
                for _ in range(7):
                    d.t -= 0.1
                for _ in range(7):
                    d.t += 0.1
                    state = d.race.frame(d.t, False, False)
                self.assertEqual(state['reason'], 'odometry_timeout')
            self.assertEqual((state['phase'], state['action']), ('fault', 'stop'))
            self.assertEqual(d.step()['phase'], 'fault')

    def test_arm_requires_both_clear_and_fresh(self):
        d = Driver()
        with self.assertRaises(ValueError):
            d.race.arm(d.t)
        d.repeat(3, line=True)
        with self.assertRaises(ValueError):
            d.race.arm(d.t)
        d.repeat(2)
        self.assertEqual(d.race.arm(d.t)['phase'], 'armed')


class CommandTests(unittest.TestCase):
    def test_disabled_and_watchdog_and_clamps(self):
        state = {'calibrated': True, 'phase': 'running', 'action': 'follow_lane'}
        self.assertIsNone(CommandGate(0.3, 0.4).output(1))
        gate = CommandGate(0.3, 0.4, enabled=True)
        self.assertEqual(gate.output(1), (0, 0, 0))
        gate.set_state(state, 1)
        gate.set_command(3, 4, 3, 1)
        self.assertEqual(gate.output(1.1), (0, 0, 0))
        gate.set_safety(True, 1)
        vx, vy, w = gate.output(1.1)
        self.assertAlmostEqual(math.hypot(vx, vy), 0.3)
        self.assertEqual(w, 0.4)
        self.assertEqual(gate.output(1.6), (0, 0, 0))
        gate.set_safety(False, 1.1)
        self.assertEqual(gate.output(1.1), (0, 0, 0))
        gate.set_safety(True, 1.1)
        gate.set_state({'calibrated': True, 'phase': 'yield', 'action': 'stop'}, 1.1)
        self.assertEqual(gate.output(1.1), (0, 0, 0))
        gate.set_state(state, 1.1)
        gate.set_command(float('nan'), 0, 0, 1.1)
        self.assertEqual(gate.output(1.1), (0, 0, 0))
        gate.set_command(0.2, 0, 0, 1.2)
        gate.set_state(None, 1.2)
        self.assertEqual(gate.output(1.2), (0, 0, 0))

    def test_extreme_components_cannot_generate_nonfinite_output(self):
        gate = CommandGate(.3, .4, enabled=True)
        gate.set_state({'calibrated': True, 'phase': 'running', 'action': 'follow_lane'}, 1)
        gate.set_safety(True, 1)
        gate.set_command(1.7e308, 1.7e308, 0, 1)
        self.assertEqual(gate.output(1), (0, 0, 0))


class VoiceTests(unittest.TestCase):
    def test_only_matching_handshake_and_vendor_stop_are_used(self):
        self.assertEqual(firmware_reply(b'xx\xaa\x55\x01\x0a\x55\xaa'),1.0)
        self.assertIsNone(firmware_reply(b'\xaa\x55\x02\x0a\x55\xaa'))
        class Port:
            def __init__(self): self.writes=[]
            def reset_input_buffer(self): pass
            def write(self,data): self.writes.append(data); return len(data)
            def flush(self): pass
            def read(self,n): return b'\xaa\x55\x01\x0a\x55\xaa'
        port=Port()
        voice=VoiceBox(port)
        self.assertEqual(voice.handshake(),1.0)
        voice.play_stop()
        self.assertEqual(port.writes,[HANDSHAKE,STOP_PROMPT])


class BaseProtocolTests(unittest.TestCase):
    def test_velocity_command_and_input_validation(self):
        command=velocity_frame(.12,-.03,.4)
        self.assertEqual(command[:4],bytes([0xab,0xcd,11,0x11]))
        self.assertEqual(struct.unpack('>hhh',command[4:10]),(120,-30,400))
        self.assertEqual(sum(command[:-1])&255,command[-1])
        with self.assertRaises(ValueError):velocity_frame(float('nan'),0,0)
    def test_fragmented_sensor_data_and_checksum(self):
        data=bytearray(32)
        data[:3]=bytes([0xab,0xcd,32])
        data[16:22]=struct.pack('>hhh',125,-20,7)
        data[22:26]=struct.pack('>hh',1240,423)
        data[26:30]=bytes([1,0,2,1])
        data[-1]=sum(data[:-1])&255
        broken=bytearray(data)
        broken[-1]^=1
        decoder=FrameDecoder()
        self.assertEqual(decoder.feed(b'noise'+bytes(data[:10])),[])
        frames=decoder.feed(bytes(data[10:])+broken+data)
        self.assertEqual(len(frames),2)
        self.assertEqual(measured_velocity(frames[0]),(.125,-.02,.007))
        self.assertEqual(measured_sensors(frames[0]),{'battery_vendor_value':12.4,
                         'sonar_vendor_value':42.3, 'infrared_vendor_values':[1,0,2,1]})
        with self.assertRaises(ValueError):measured_sensors(bytes(broken))


class CameraCalibrationTests(unittest.TestCase):
    def test_calibration_validation(self):
        value={'image_width':640,'image_height':480,'camera_matrix':{'data':[400,0,320,0,400,240,0,0,1]},
               'distortion_coefficients':{'data':[0,0,0,0,0]},'rectification_matrix':{'data':[1,0,0,0,1,0,0,0,1]},
               'projection_matrix':{'data':[400,0,320,0,0,400,240,0,0,0,1,0]}}
        self.assertEqual(read_calibration(value)['width'],640)
        value['camera_matrix']['data'][0]=float('nan')
        with self.assertRaises(ValueError):read_calibration(value)
    def test_calibration_sample_diversity(self):
        points=[(100+x*20,100+y*20) for y in range(6) for x in range(8)]
        first=signature(points,8,6,640,480)
        self.assertFalse(novel(first,[first]))
        shifted=signature([(x+100,y) for x,y in points],8,6,640,480)
        self.assertTrue(novel(shifted,[first]))


try:
    import numpy as np
    from PIL import Image
    from automation.detector import detect, validate_config
    from automation.yolo import predictions
    from ros2.automatic_race import decode, source_stamp_fresh
except ImportError:
    np = None


@unittest.skipIf(np is None, 'numpy and Pillow are needed for image tests')
class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((Path(__file__).resolve().parents[1] / 'data/perception.example.json').read_text())
    def test_synthetic_line_and_crosswalk(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        self.assertFalse(detect(image, self.config)['start_line']['detected'])
        image[80:84, 10:90] = 255
        for x in (15, 30, 45, 60, 75):
            image[42:75, x:x+5] = 255
        results = detect(image, self.config)
        self.assertTrue(results['start_line']['detected'])
        self.assertTrue(results['crosswalk']['detected'])
        self.assertEqual(results['crosswalk']['bands'], 5)
    def test_config_rejected_and_padded_raw_decode(self):
        self.config['start_line']['roi'] = [0.5, 0, 0.1, 1]
        with self.assertRaises(ValueError):
            validate_config(self.config)
        class Message:
            encoding = 'bgr8'
            width, height, step = 2, 1, 8
            data = bytes([1, 2, 3, 4, 5, 6, 99, 99])
        self.assertEqual(decode(Message(), 'raw').tolist(), [[[3, 2, 1], [6, 5, 4]]])
    def test_reject_old_missing_or_future_source_stamps(self):
        class Stamp: sec, nanosec = 10, 0
        class Header: stamp = Stamp()
        self.assertTrue(source_stamp_fresh(Header(), 10_500_000_000, .8))
        self.assertFalse(source_stamp_fresh(Header(), 11_000_000_000, .8))
        self.assertFalse(source_stamp_fresh(Header(), 9_500_000_000, .8))
        Stamp.sec = 0
        self.assertFalse(source_stamp_fresh(Header(), 10_000_000_000, .8))
    def test_race_model_class_scores_and_per_class_suppression(self):
        output=np.array([[320,320,100,100,.9,.8,.1],[322,321,100,100,.8,.8,.1],
                         [320,320,100,100,.9,.1,.9],[float('nan'),0,0,0,0,0,0]],dtype=np.float32)
        result=predictions(output,640,480)
        self.assertEqual([item['label'] for item in result],['start','crossing'])
        self.assertEqual(result[1]['score'],.72)
        mapped=predictions(output,640,480,class_names=['start','crossing'])
        self.assertEqual([x['label'] for x in mapped],['crossing','start'])
        self.assertEqual([x['class_id'] for x in mapped],[1,0])


if __name__ == '__main__':
    unittest.main()
