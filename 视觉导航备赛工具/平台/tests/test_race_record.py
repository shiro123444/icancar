import json
import tempfile
import unittest
from pathlib import Path
from race_record import RaceRecords,PENALTY_FIELDS

class FakeClock:
 def __init__(self):self.time=100.0
 def __call__(self):return self.time
 def set(self,t):self.time=t

class RaceRecordsTest(unittest.TestCase):
 def test_two_laps_parking_and_review(self):
  with tempfile.TemporaryDirectory() as tmp:
   c=FakeClock();r=RaceRecords(tmp,clock=c,wall=lambda:'2026-10-28T12:00:00+08:00')
   s=r.event('start');self.assertEqual(s['phase'],'running');self.assertIsNone(s['penalty_total'])
   c.set(101);r.event('park_start');c.set(104.25);self.assertEqual(r.event('park_end')['record']['parking'][0]['duration_s'],3.25)
   c.set(110);self.assertEqual(r.event('lap')['record']['lap_times_s'],[10.0])
   c.set(112);r.event('park_start');c.set(115.5);r.event('park_end');c.set(120);s=r.event('lap')
   self.assertEqual(s['phase'],'finished');self.assertEqual(s['manual_elapsed_s'],20.0)
   penalty={k:'0' for k in PENALTY_FIELDS};penalty['boundary']='10';penalty['crosswalk1']='10'
   s=r.review({'video_time_s':'20.32','penalties':penalty,'broadcast':['yes','no'],'notes':'回看'})
   self.assertEqual(s['penalty_total'],'20.00');self.assertEqual(s['effective_time'],'40.32')
   saved=json.loads(next(Path(tmp).glob('*.json')).read_text());self.assertEqual(saved['notes'],'回看')
   with self.assertRaises(ValueError):r.event('lap')
 def test_no_implicit_zero_and_validation_is_transactional(self):
  with tempfile.TemporaryDirectory() as tmp:
   r=RaceRecords(tmp);r.event('start');p={k:'' for k in PENALTY_FIELDS};p['barrier']='10'
   self.assertIsNone(r.review({'video_time_s':'42.00','penalties':p})['effective_time'])
   p['boundary']='-10'
   with self.assertRaises(ValueError):r.review({'penalties':p})
   self.assertEqual(r.snapshot()['record']['penalties']['barrier'],'10.00')
   for invalid in ('nan','1.001','-1','inf'):
    p['boundary']=invalid
    with self.assertRaises(ValueError):r.review({'penalties':p})
 def test_restart_running_marks_interrupted_and_preserves_history(self):
  with tempfile.TemporaryDirectory() as tmp:
   c=FakeClock();a=RaceRecords(tmp,clock=c);first=a.event('start')['record']['id'];c.set(105);a.event('park_start')
   b=RaceRecords(tmp,clock=c);self.assertEqual(b.snapshot()['phase'],'interrupted');self.assertEqual(b.snapshot()['record']['last_marked_elapsed_s'],5)
   with self.assertRaises(ValueError):b.event('lap')
   second=b.event('start')['record']['id'];self.assertNotEqual(first,second)
   with self.assertRaises(ValueError):b.select(first)
   b.close();self.assertEqual(b.select(first)['record']['id'],first);self.assertEqual(len(b.history()),2)
 def test_misordered_parking_and_extra_laps(self):
  with tempfile.TemporaryDirectory() as tmp:
   c=FakeClock();r=RaceRecords(tmp,clock=c);r.event('start')
   with self.assertRaises(ValueError):r.event('park_end')
   r.event('park_start')
   with self.assertRaises(ValueError):r.event('lap')
   r.event('park_clear');r.event('lap');r.event('lap')
   with self.assertRaises(ValueError):r.event('lap')
   with self.assertRaises(ValueError):r.select('../etc/passwd')
if __name__=='__main__':unittest.main()
