import csv
import importlib.util
import io
import tempfile
import unittest
from pathlib import Path

try:
    from PIL import Image
    from training.prepare_vendor_dataset import prepare
    import torch
    from training.train_steering import AutoPilot, DrivingDataset
    READY = True
except ImportError:
    READY = False


@unittest.skipUnless(READY, 'training dependencies are not installed')
class TrainingPreparationTests(unittest.TestCase):
    def test_portable_time_split_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            source, output = base / 'source', base / 'prepared'
            source.mkdir()
            with (source / 'data.csv').open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['image_path', 'steering'])
                for index in range(30):
                    name = str(1_000_000 + index * 1000) + '.jpg'
                    Image.new('RGB', (24, 24), (index, 0, 0)).save(source / name)
                    writer.writerow(['/old/car/path/' + name, index / 100])
            meta = prepare(source, output, .2, 3)
            self.assertEqual((meta['samples'], meta['training_samples'], meta['validation_samples'], meta['gap_samples']),
                             (30, 21, 6, 3))
            train = DrivingDataset(output, 'train.csv')
            validation = DrivingDataset(output, 'validation.csv')
            self.assertEqual(len(train), 21)
            self.assertEqual(len(validation), 6)
            x, y = train[0]
            self.assertEqual(tuple(x.shape), (3, 120, 160))
            self.assertEqual(tuple(y.shape), (1,))
            self.assertTrue({p for p, _ in train.samples}.isdisjoint({p for p, _ in validation.samples}))
            with self.assertRaises(ValueError):
                prepare(source, output)
            holdout = base / 'holdout'
            held = prepare(source, holdout, .2, 1, .2)
            self.assertEqual((held['training_samples'], held['validation_samples'], held['test_samples'], held['gap_samples']),
                             (17, 5, 6, 2))
            self.assertEqual(len(DrivingDataset(holdout, 'test.csv')), 6)

    def test_model_shape_and_compatible_state_dictionary(self):
        model = AutoPilot()
        self.assertEqual(tuple(model(torch.zeros(2, 3, 120, 160)).shape), (2, 1))
        self.assertEqual(tuple(model.state_dict()['fc.0.weight'].shape), (100, 9792))
        self.assertEqual(set(model.state_dict()), {
            'cnn.0.weight', 'cnn.0.bias', 'cnn.2.weight', 'cnn.2.bias', 'cnn.4.weight', 'cnn.4.bias',
            'fc.0.weight', 'fc.0.bias', 'fc.2.weight', 'fc.2.bias', 'fc.4.weight', 'fc.4.bias'})

    def test_preview_loads_weights_and_rejects_bad_images(self):
        spec = importlib.util.spec_from_file_location('steering_preview', Path(__file__).resolve().parents[1] / 'ros2' / 'steering_preview.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'model.pth'
            torch.save(AutoPilot().state_dict(), path)
            model = module.load_model(path)
            image = io.BytesIO()
            Image.new('RGB', (24, 24), 'white').save(image, format='JPEG')
            steering, inference_ms = module.predict_jpeg(model, image.getvalue())
            self.assertTrue(-100 < steering < 100)
            self.assertGreaterEqual(inference_ms, 0)
            with self.assertRaises(ValueError):
                module.predict_jpeg(model, b'not an image')


if __name__ == '__main__':
    unittest.main()
