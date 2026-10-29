import sys
import tempfile
import time
import unittest
from pathlib import Path
from workflow import TrainingJob

class WorkflowTest(unittest.TestCase):
 def wait_for(self,job,predicate,timeout=5):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   state=job.status()
   if predicate(state):return state
   time.sleep(.02)
  self.fail('任务状态超时：'+repr(job.status()))
 def test_train_and_no_implicit_dataset_change(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);data=p/'data.csv';data.write_text('image,value\nmissing.jpg,1\n');script=p/'train.py';script.write_text('print("epoch 1", flush=True)\n')
   job=TrainingJob();job.start(sys.executable,str(script),str(data));state=self.wait_for(job,lambda s:s['state']=='completed')
   self.assertEqual(state['exit_code'],0);self.assertIn('epoch 1','\n'.join(state['logs']));self.assertEqual(data.read_text(),'image,value\nmissing.jpg,1\n')
 def test_reject_overlapping_and_stop(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);data=p/'data.csv';data.write_text('image,value\nx.jpg,1\n');script=p/'train.py';script.write_text('import time\nprint("ready",flush=True)\ntime.sleep(60)\n')
   job=TrainingJob();job.start(sys.executable,str(script),str(data));self.wait_for(job,lambda s:'ready' in s['logs'])
   with self.assertRaises(ValueError):job.start(sys.executable,str(script),str(data))
   job.stop();state=self.wait_for(job,lambda s:s['state']=='failed');self.assertNotEqual(state['exit_code'],0)
 def test_reject_missing_or_relative_paths(self):
  with self.assertRaises(ValueError):TrainingJob().start(sys.executable,'train.py','data.csv')
if __name__=='__main__':unittest.main()
