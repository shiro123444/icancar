import csv
import tempfile
import unittest
from pathlib import Path
from ros2.dataset_session import DatasetSession
class DatasetSessionTest(unittest.TestCase):
 def test_freshness_and_non_overwrite(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/'session';s=DatasetSession(p,{'format':'test'},min_interval_s=.2,max_command_age_s=.25)
   try:
    self.assertFalse(s.add_frame(b'\xff\xd8x',10,1))
    s.set_command(.2,.1,-.3,1.1)
    self.assertTrue(s.add_frame(b'\xff\xd8x',11,1.2))
    self.assertFalse(s.add_frame(b'\xff\xd8x',12,1.3))
    self.assertFalse(s.add_frame(b'\xff\xd8x',13,1.5))
    s.set_command(.3,0,0,1.5);self.assertTrue(s.add_frame(b'\xff\xd8x',14,1.51))
   finally:s.close()
   with (p/'data.csv').open() as f:rows=list(csv.DictReader(f))
   self.assertEqual(len(rows),2);self.assertEqual(rows[0]['image'],'images/frame_000001.jpg');self.assertEqual(rows[0]['angular_z'],'-0.3')
   self.assertEqual(s.skipped_stale,2)
   with self.assertRaises(FileExistsError):DatasetSession(p,{})
 def test_invalid_inputs(self):
  with tempfile.TemporaryDirectory() as tmp:
   s=DatasetSession(Path(tmp)/'one',{})
   try:
    with self.assertRaises(ValueError):s.set_command(float('nan'),0,0,0)
    s.set_command(0,0,0,0)
    with self.assertRaises(ValueError):s.add_frame(b'not jpeg',0,0)
   finally:s.close()
if __name__=='__main__':unittest.main()
