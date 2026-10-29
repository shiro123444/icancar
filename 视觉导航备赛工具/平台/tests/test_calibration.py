import base64
import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from pathlib import Path

import server
from calibration import CalibrationSessions
from race_record import RaceRecords

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / '.venv-calibration' / 'bin' / 'python'
WORKER = ROOT / 'tools' / 'calibration_worker.py'


class CalibrationStorageTest(unittest.TestCase):
    def test_saved_photos_survive_restart_and_can_be_excluded(self):
        # The image is intentionally checked later, when the compute environment exists.
        image = b'\x89PNG\r\n\x1a\n' + b'placeholder'
        with tempfile.TemporaryDirectory() as temp:
            manager = CalibrationSessions(temp, Path(temp) / 'missing-python', WORKER)
            session = manager.create(8, 6, 20)
            session = manager.add_photo(session['id'], image, 'import')
            name = session['photos'][0]['file']
            self.assertIsNone(session['photos'][0]['detected'])
            self.assertIsNone(session['photos'][0]['captured_at'])
            self.assertIsNotNone(session['photos'][0]['saved_at'])
            restarted = CalibrationSessions(temp, Path(temp) / 'missing-python', WORKER)
            self.assertEqual(restarted.photo(session['id'], name)[0], image)
            revised = restarted.set_enabled(session['id'], name, False)
            self.assertFalse(revised['photos'][0]['enabled'])
            self.assertEqual(restarted.photo(session['id'], name)[0], image)
            with zipfile.ZipFile(BytesIO(restarted.export(session['id']))) as archive:
                self.assertEqual(archive.read(name), image)
                self.assertIn('session.json', archive.namelist())
            with self.assertRaises(ValueError):
                restarted.photo(session['id'], '../session.json')
            with self.assertRaises(ValueError):
                restarted.detail('../outside')


class CalibrationApiTest(unittest.TestCase):
    def test_manual_capture_and_stale_camera(self):
        class Bridge(BaseHTTPRequestHandler):
            age = 0.1
            def log_message(self, *args): pass
            def do_GET(self):
                data = (json.dumps({'camera_age_s': self.age}).encode() if self.path == '/health'
                        else b'\xff\xd8\xff' + b'fake-frame')
                self.send_response(200); self.send_header('Content-Length', str(len(data)))
                self.end_headers(); self.wfile.write(data)
        with tempfile.TemporaryDirectory() as temp:
            bridge = ThreadingHTTPServer(('127.0.0.1', 0), Bridge)
            api = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
            api.telemetry_port = bridge.server_port
            api.records = RaceRecords(Path(temp) / 'records')
            api.calibrations = CalibrationSessions(Path(temp) / 'sessions', Path(temp) / 'missing', WORKER)
            for item in (bridge, api):
                threading.Thread(target=item.serve_forever, daemon=True).start()
            url = 'http://127.0.0.1:%d' % api.server_port
            def post(path, data, extra=None):
                headers = {'Content-Type': 'application/json'}
                headers.update(extra or {})
                request = urllib.request.Request(url + path, json.dumps(data).encode(), headers, method='POST')
                return json.load(urllib.request.urlopen(request))
            try:
                created = post('/api/calibration/create', {'cols': 8, 'rows': 6, 'square_mm': 20})
                self.assertEqual(len(created['photos']), 0)
                saved = post('/api/calibration/capture', {'id': created['id']})
                self.assertEqual(len(saved['photos']), 1)
                self.assertIsNotNone(saved['photos'][0]['captured_at'])
                Bridge.age = 10
                with self.assertRaises(urllib.error.HTTPError):
                    post('/api/calibration/capture', {'id': created['id']})
                latest = json.load(urllib.request.urlopen(url + '/api/calibration/session?id=' + created['id']))
                self.assertEqual(len(latest['photos']), 1)
                with self.assertRaises(urllib.error.HTTPError) as blocked:
                    post('/api/calibration/create', {'cols': 8, 'rows': 6, 'square_mm': 20}, {'Origin': 'https://other.example'})
                self.assertEqual(blocked.exception.code, 403)
            finally:
                for item in (api, bridge):
                    item.shutdown(); item.server_close()


try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipUnless(cv2 is not None and PYTHON.is_file(), 'requires calibration environment')
class CalibrationCalculationTest(unittest.TestCase):
    def test_saved_photos_compute_offline_and_result_becomes_stale(self):
        import math
        import yaml
        texture = np.full((420, 540), 255, np.uint8)
        for y in range(7):
            for x in range(9):
                if (x + y) % 2 == 0:
                    texture[y*60:(y+1)*60, x*60:(x+1)*60] = 0
        source = np.float32([[0, 0], [540, 0], [540, 420], [0, 420]])
        board = np.float32([[0, 0, 0], [.18, 0, 0], [.18, .14, 0], [0, .14, 0]])
        k = np.float64([[550, 0, 320], [0, 550, 240], [0, 0, 1]])
        with tempfile.TemporaryDirectory() as temp:
            manager = CalibrationSessions(temp, PYTHON, WORKER)
            session = manager.create(8, 6, 20)
            for i in range(18):
                r = np.float64([.24 * math.sin(i*.73), .25 * math.cos(i*.83), .12 * math.sin(i*.41)])
                t = np.float64([-.09 + .04 * math.sin(i*.57), -.07 + .03 * math.cos(i*.64), .48 + .09 * math.sin(i*.38)])
                target, _ = cv2.projectPoints(board, r, t, k, np.zeros(5))
                transform = cv2.getPerspectiveTransform(source, np.float32(target.reshape(4, 2)))
                image = cv2.warpPerspective(texture, transform, (640, 480), borderValue=255)
                ok, encoded = cv2.imencode('.jpg', image)
                self.assertTrue(ok)
                session = manager.add_photo(session['id'], encoded.tobytes(), 'import')
            self.assertGreaterEqual(sum(p['detected'] is True for p in session['photos']), 12)
            session = manager.compute(session['id'])
            result = session['results'][-1]
            self.assertTrue(result['current'])
            self.assertGreaterEqual(result['sample_count'], 12)
            self.assertLess(result['rms_px'], 1)
            report = yaml.safe_load(manager.result_file(session['id'], result['file']))
            self.assertAlmostEqual(report['camera_matrix']['data'][0], 550, delta=70)
            changed = manager.set_enabled(session['id'], session['photos'][0]['file'], False)
            self.assertFalse(changed['results'][-1]['current'])


if __name__ == '__main__':
    unittest.main()
