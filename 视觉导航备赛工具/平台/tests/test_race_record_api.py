import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from race_record import RaceRecords,PENALTY_FIELDS,COUNT_FIELDS
import server

class RecordAPITest(unittest.TestCase):
 def test_record_review_export_and_host_guard(self):
  with tempfile.TemporaryDirectory() as temp:
   http=ThreadingHTTPServer(('127.0.0.1',0),server.Handler);http.telemetry_port=1;http.records=RaceRecords(Path(temp)/'records')
   worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start();base='http://127.0.0.1:'+str(http.server_port)
   def get(path,headers=None):return json.load(urllib.request.urlopen(urllib.request.Request(base+path,headers=headers or {})))
   def post(path,data,origin=None):
    headers={'Content-Type':'application/json'}
    if origin:headers['Origin']=origin
    return json.load(urllib.request.urlopen(urllib.request.Request(base+path,data=json.dumps(data).encode(),headers=headers,method='POST')))
   try:
    self.assertEqual(get('/api/race-record/current')['phase'],'idle')
    with self.assertRaises(urllib.error.HTTPError) as bad:post('/api/race-record/event',{'action':'start'},'https://example.com')
    self.assertEqual(bad.exception.code,403)
    with self.assertRaises(urllib.error.HTTPError) as bad:get('/api/race-record/current',{'Host':'example.com'})
    self.assertEqual(bad.exception.code,403)
    run_id=post('/api/race-record/event',{'action':'start'})['record']['id']
    with self.assertRaises(urllib.error.HTTPError):post('/api/race-record/event',{'action':'park_start','id':'run-'+'0'*32})
    post('/api/race-record/event',{'action':'park_start','id':run_id});post('/api/race-record/event',{'action':'park_end','id':run_id})
    post('/api/race-record/event',{'action':'lap','id':run_id});s=post('/api/race-record/event',{'action':'lap','id':run_id});self.assertEqual(s['phase'],'finished')
    payload={'id':run_id,'video_time_s':'70.25','video_parking_s':['3.2','3.5'],'counts':{k:'0' for k in COUNT_FIELDS},'broadcast':['yes','yes'],'penalties':{k:'0' for k in PENALTY_FIELDS},'notes':'tested'}
    s=post('/api/race-record/review',payload);self.assertEqual(s['effective_time'],'70.25');self.assertEqual(s['record']['video_parking_s'],['3.20','3.50'])
    self.assertEqual(len(get('/api/race-record/history')['records']),1)
    self.assertEqual(get('/api/race-record/export')['record']['notes'],'tested')
   finally:http.records.close();http.shutdown();http.server_close();worker.join()
if __name__=='__main__':unittest.main()
