import json
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
import urllib.error
from pathlib import Path
from http.server import ThreadingHTTPServer
import server

class WorkflowAPITest(unittest.TestCase):
 def test_inspection_training_and_origin_guard(self):
  http=ThreadingHTTPServer(('127.0.0.1',0),server.Handler);http.telemetry_port=1
  worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start();base='http://127.0.0.1:'+str(http.server_port)
  def post(path,obj,origin=None):
   headers={'Content-Type':'application/json'}
   if origin:headers['Origin']=origin
   return json.load(urllib.request.urlopen(urllib.request.Request(base+path,data=json.dumps(obj).encode(),headers=headers,method='POST')))
  try:
   with tempfile.TemporaryDirectory() as temp:
    p=Path(temp);(p/'x.jpg').write_bytes(b'test');csv=p/'data.csv';csv.write_text('image,steering\nx.jpg,0\n');script=p/'train.py';script.write_text('print("training done")\n')
    self.assertEqual(post('/api/workflow/audit',{'csv':str(csv)})['existing_image_paths'],1)
    with self.assertRaises(urllib.error.HTTPError) as err:post('/api/workflow/train',{'csv':str(csv),'script':str(script),'interpreter':sys.executable},'http://example.com')
    self.assertEqual(err.exception.code,403)
    self.assertEqual(post('/api/workflow/train',{'csv':str(csv),'script':str(script),'interpreter':sys.executable})['state'],'running')
    for _ in range(100):
     status=json.load(urllib.request.urlopen(base+'/api/workflow/status'))
     if status['state']!='running':break
     time.sleep(.02)
    self.assertEqual(status['state'],'completed');self.assertIn('training done','\n'.join(status['logs']))
  finally:http.shutdown();http.server_close();worker.join()
if __name__=='__main__':unittest.main()
