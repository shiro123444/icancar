import importlib.util
import json
import math
import threading
import unittest
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
from pathlib import Path
spec=importlib.util.spec_from_file_location('telemetry',Path(__file__).resolve().parents[1]/'ros2'/'telemetry_bridge.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class BridgeTest(unittest.TestCase):
    def test_yaw(self):
        self.assertAlmostEqual(m.yaw_from_quaternion(0,0,math.sin(math.pi/4),math.cos(math.pi/4)),math.pi/2)
    def test_stale_camera(self):
        now=[10.0];store=m.Store(lambda:now[0]);store.set_frame(b'\xff\xd8test')
        self.assertIsNotNone(store.image());now[0]=13.01;self.assertIsNone(store.image())
        self.assertEqual(store.status()['camera_age_s'],3.01)
    def test_mission_status_and_bad_json(self):
        now=[10.0];store=m.Store(lambda:now[0])
        state=m.parse_mission('{"phase":"yield","laps":1,"crosswalk_stops":1,"elapsed_seconds":12.3,"stationary_seconds":1.0}')
        store.set_mission(state);now[0]=11.0
        self.assertEqual(store.status()['mission']['phase'],'yield')
        self.assertEqual(store.status()['mission_age_s'],1)
        with self.assertRaises(ValueError): m.parse_mission('{"phase":"running","elapsed_seconds":NaN}')
    def test_model_status_validation(self):
        data={'source':'openvino_race','width':640,'height':480,'inference_ms':20,
              'detections':[{'label':'crossing','score':.8,'box':[10,20,100,150]}]}
        value=m.parse_model(json.dumps(data));store=m.Store();store.set_model(value)
        self.assertEqual(store.status()['model']['detections'][0]['label'],'crossing')
        data['detections'][0]['box'][2]=900
        with self.assertRaises(ValueError):m.parse_model(json.dumps(data))
    def test_calibration_status_validation(self):
        value=m.parse_calibration('{"state":"collecting","accepted":3,"target":18,"board_visible":true}')
        store=m.Store();store.set_calibration(value)
        self.assertEqual(store.status()['calibration']['accepted'],3)
        with self.assertRaises(ValueError):m.parse_calibration('{"state":"collecting","accepted":19,"target":18,"board_visible":true}')
    def test_steering_preview_validation_and_age(self):
        now=[10.0];store=m.Store(lambda:now[0])
        value=m.parse_steering('{"source":"steering_cnn","valid":true,"steering":-0.31,"inference_ms":12}')
        store.set_steering(value);now[0]=11.5
        self.assertEqual(store.status()['steering']['steering'],-0.31)
        self.assertEqual(store.status()['steering_age_s'],1.5)
        self.assertFalse(m.parse_steering('{"source":"steering_cnn","valid":false}')['valid'])
        timed=m.parse_steering('{"source":"steering_cnn","valid":true,"steering":0.1,"inference_ms":2,"processing_ms":6,"image_age_ms":11,"image_stamp_ns":1000000000}')
        self.assertEqual((timed['processing_ms'],timed['image_age_ms'],timed['image_stamp_ns']),(6,11,1000000000))
        with self.assertRaises(ValueError):m.parse_steering('{"source":"steering_cnn","valid":true,"steering":0,"inference_ms":1,"processing_ms":-1}')
        with self.assertRaises(ValueError):m.parse_steering('{"source":"steering_cnn","valid":true,"steering":NaN,"inference_ms":12}')
        with self.assertRaises(ValueError):m.parse_steering('{"source":"steering_cnn","valid":true,"steering":0,"inference_ms":-1}')
    def test_http_is_read_only(self):
        store=m.Store();store.set_pose({'x':1,'y':2,'yaw_rad':0,'frame_id':'odom','child_frame_id':'base_link'})
        server=ThreadingHTTPServer(('127.0.0.1',0),m.handler_for(store))
        t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
        try:
            url='http://127.0.0.1:'+str(server.server_port)
            self.assertEqual(json.load(urllib.request.urlopen(url+'/health'))['pose']['x'],1)
            with self.assertRaises(urllib.error.HTTPError) as e: urllib.request.urlopen(url+'/snapshot.jpg')
            self.assertEqual(e.exception.code,503)
            with self.assertRaises(urllib.error.HTTPError): urllib.request.urlopen(urllib.request.Request(url+'/command',data=b'{}',method='POST'))
        finally: server.shutdown();server.server_close();t.join()

    def test_control_diagnostics_are_validated_and_expire_in_status(self):
        now=[10.0];store=m.Store(lambda:now[0])
        value=m.parse_control('{"source":"control_gate","active":true,"reason":"estop_latched_or_stale","estop_latched":true,"feedback_fault":null,"feedback_age_s":0.1}')
        store.set_control(value);now[0]=11.0
        self.assertEqual(store.status()['control']['reason'],'estop_latched_or_stale')
        self.assertEqual(store.status()['control_age_s'],1.0)
        with self.assertRaises(ValueError):m.parse_control('{"source":"control_gate","active":true,"reason":"ready","estop_latched":true,"feedback_age_s":NaN}')
if __name__=='__main__':unittest.main()
