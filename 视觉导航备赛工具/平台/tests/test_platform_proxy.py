import importlib.util
import json
import threading
import unittest
import urllib.request
import urllib.error
from http.server import ThreadingHTTPServer
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod
server_mod=load('platform_server',root/'server.py')
bridge_mod=load('telemetry_bridge',root/'ros2'/'telemetry_bridge.py')
class ProxyTest(unittest.TestCase):
 def test_proxy_status_camera_and_no_control(self):
  store=bridge_mod.Store();store.set_pose({'x':2,'y':3,'yaw_rad':0,'frame_id':'odom','child_frame_id':'base_link'});store.set_frame(b'\xff\xd8test')
  bridge=ThreadingHTTPServer(('127.0.0.1',0),bridge_mod.handler_for(store));threading.Thread(target=bridge.serve_forever,daemon=True).start()
  platform=ThreadingHTTPServer(('127.0.0.1',0),server_mod.Handler);platform.telemetry_port=bridge.server_port;threading.Thread(target=platform.serve_forever,daemon=True).start()
  try:
   base='http://127.0.0.1:'+str(platform.server_port)
   self.assertEqual(json.load(urllib.request.urlopen(base+'/api/robot/telemetry'))['pose']['x'],2)
   self.assertEqual(urllib.request.urlopen(base+'/api/robot/camera').read(),b'\xff\xd8test')
   with self.assertRaises(urllib.error.HTTPError):urllib.request.urlopen(urllib.request.Request(base+'/api/robot/command',data=b'{}',method='POST'))
  finally:platform.shutdown();platform.server_close();bridge.shutdown();bridge.server_close()
if __name__=='__main__':unittest.main()
