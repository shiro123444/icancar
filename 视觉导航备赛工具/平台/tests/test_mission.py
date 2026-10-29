import unittest
from mission.task2 import Mission
class MissionTest(unittest.TestCase):
 def test_start_gate_edges_and_two_laps(self):
  m=Mission();m.process('start',0);self.assertEqual(m.process('gate_enter',1)['laps'],0)
  m.process('gate_clear',2);self.assertEqual(m.process('gate_enter',3)['laps'],1)
  self.assertEqual(m.process('gate_enter',4)['laps'],1)
  m.process('gate_clear',5);s=m.process('gate_enter',6)
  self.assertEqual(s['laps'],2);self.assertEqual(s['phase'],'finished');self.assertEqual(s['action'],'stop');self.assertEqual(s['elapsed_seconds'],6)
 def test_crosswalk_stationary_requires_fresh_continuous_feedback(self):
  m=Mission();m.process('start',0);s=m.process('crosswalk',1)
  self.assertEqual(s['action'],'stop');self.assertTrue(s['announcement'])
  m.process('motion',1.1,0);self.assertEqual(m.process('tick',5)['phase'],'yield')
  for i in range(13):s=m.process('motion',5.1+i*.25,0)
  self.assertEqual(s['phase'],'running');self.assertEqual(s['crosswalk_stops_completed'],1)
  self.assertEqual(m.process('crosswalk',8.2)['phase'],'running')
  m.process('crosswalk_clear',8.3);self.assertEqual(m.process('crosswalk',8.4)['phase'],'yield')
 def test_moving_feedback_resets_and_obstacle_holds(self):
  m=Mission();m.process('start',0);m.process('obstacle',.1);self.assertEqual(m.snapshot(.1)['action'],'stop')
  m.process('crosswalk',.2);m.process('motion',.3,0);m.process('motion',.5,.2)
  for i in range(13):s=m.process('motion',.6+i*.25,0)
  self.assertEqual(s['phase'],'running');self.assertEqual(s['action'],'stop')
  self.assertEqual(m.process('obstacle_clear',3.7)['action'],'follow_lane')
 def test_invalid_time_and_values(self):
  m=Mission();m.process('start',1)
  with self.assertRaises(ValueError):m.process('tick',.5)
  with self.assertRaises(ValueError):m.process('motion',2,float('nan'))
  with self.assertRaises(ValueError):Mission(stop_seconds=2)
if __name__=='__main__':unittest.main()

class MissionDemoTest(unittest.TestCase):
 def test_demo_completes_two_stops_and_two_laps(self):
  from mission.demo import demo
  d=demo();self.assertEqual(d['result']['phase'],'finished');self.assertEqual(d['result']['laps'],2);self.assertEqual(d['result']['crosswalk_stops_completed'],2)
  self.assertTrue(any(e['event']=='检测到障碍' and e['action']=='stop' for e in d['timeline']))
