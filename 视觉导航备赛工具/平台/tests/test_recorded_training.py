import csv
import io
import tempfile
import unittest
from pathlib import Path

try:
    from PIL import Image
    from ros2.dataset_session import DatasetSession
    from training.prepare_recorded_dataset import prepare
    READY = True
except ImportError:
    READY = False


@unittest.skipUnless(READY, 'training image dependencies are not installed')
class RecordedTrainingTests(unittest.TestCase):
    def test_recorder_output_is_prepared_without_overwriting(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, output = root / 'recording', root / 'prepared'
            session = DatasetSession(source, {'camera_topic': '/camera'}, min_interval_s=.2)
            picture = io.BytesIO()
            Image.new('RGB', (32, 24), 'white').save(picture, 'JPEG')
            for i in range(70):
                speed = 0 if i < 4 else .1
                session.set_command(speed, 0, (i - 35) / 100, i)
                self.assertTrue(session.add_frame(picture.getvalue(), (i + 1) * 1_000_000_000, i))
            session.close()
            before = (source / 'data.csv').read_bytes()
            meta = prepare(source, output)
            self.assertEqual(meta['samples'], 66)
            self.assertEqual(meta['skipped_stop_or_reverse'], 4)
            self.assertGreaterEqual(meta['gap_samples'], 6)
            self.assertIn('angular_z command', meta['label'])
            samples = []
            for name in ('train.csv', 'validation.csv', 'test.csv'):
                with (output / name).open(newline='') as handle:
                    rows = list(csv.DictReader(handle))
                self.assertTrue(rows)
                paths = {row['image_path'] for row in rows}
                self.assertTrue(all((output / path).is_file() for path in paths))
                samples.append(paths)
            self.assertTrue(all(samples[a].isdisjoint(samples[b]) for a in range(3) for b in range(a+1, 3)))
            self.assertEqual(before, (source / 'data.csv').read_bytes())
            with self.assertRaises(ValueError):
                prepare(source, output)

    def test_bad_time_and_path_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'recording'
            source.mkdir()
            (source / 'data.csv').write_text(
                'image,linear_x,angular_z,command_age_ms,image_stamp_ns\n../outside.jpg,0.1,0,0,1\n', encoding='utf-8')
            Image.new('RGB', (16, 16)).save(root / 'outside.jpg')
            with self.assertRaises(ValueError):
                prepare(source, root / 'prepared')
            self.assertFalse((root / 'prepared').exists())


if __name__ == '__main__':
    unittest.main()
