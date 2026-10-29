"""Persistent, user-controlled camera calibration photo sessions."""
import copy
import io
import json
import math
import os
import re
import subprocess
import threading
import time
import uuid
import zipfile
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen


ID_RE = re.compile(r'^cal-[0-9]{8}-[0-9]{6}-[a-f0-9]{8}$')
PHOTO_RE = re.compile(r'^photo-[0-9]{4}\.(jpg|png)$')


class CalibrationSessions:
    def __init__(self, root, interpreter, worker):
        self.root = Path(root).resolve()
        self.interpreter = Path(interpreter)
        self.worker = Path(worker).resolve()
        self.lock = threading.RLock()
        self.root.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def now():
        return datetime.now().astimezone().isoformat(timespec='seconds')

    def directory(self, session_id):
        if not isinstance(session_id, str) or not ID_RE.fullmatch(session_id):
            raise ValueError('采集组编号无效')
        return self.root / session_id

    def _read(self, session_id):
        path = self.directory(session_id) / 'session.json'
        if not path.is_file():
            raise ValueError('找不到这组标定照片')
        return json.loads(path.read_text(encoding='utf-8'))

    def _write(self, data):
        path = self.directory(data['id']) / 'session.json'
        temp = path.with_name('.session-' + uuid.uuid4().hex + '.tmp')
        try:
            temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            os.replace(temp, path)
        finally:
            temp.unlink(missing_ok=True)

    def _detail(self, data):
        result = copy.deepcopy(data)
        result['path'] = str(self.directory(data['id']))
        for entry in result['results']:
            entry['current'] = entry['input_revision'] == data['revision']
        result['worker_ready'] = self.interpreter.is_file() and self.worker.is_file()
        return result

    def create(self, cols, rows, square_mm):
        if (isinstance(cols, bool) or isinstance(rows, bool) or not isinstance(cols, int)
                or not isinstance(rows, int) or not 3 <= cols <= 30 or not 3 <= rows <= 30):
            raise ValueError('棋盘格内角点数量应在 3 到 30 之间')
        if (isinstance(square_mm, bool) or not isinstance(square_mm, (int, float))
                or not math.isfinite(square_mm) or not 0 < square_mm <= 200):
            raise ValueError('请填写实际方格边长（毫米）')
        with self.lock:
            session_id = 'cal-' + datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8]
            self.directory(session_id).mkdir()
            data = {'id': session_id, 'created_at': self.now(),
                    'board': {'cols': cols, 'rows': rows, 'square_mm': float(square_mm)},
                    'revision': 0, 'photos': [], 'results': []}
            self._write(data)
            return self._detail(data)

    def list(self):
        with self.lock:
            result = []
            for directory in sorted(self.root.iterdir(), reverse=True):
                if not directory.is_dir() or not ID_RE.fullmatch(directory.name):
                    continue
                try:
                    result.append(self._detail(self._read(directory.name)))
                except (OSError, ValueError, json.JSONDecodeError):
                    continue
            return result

    def detail(self, session_id):
        with self.lock:
            return self._detail(self._read(session_id))

    def _worker(self, args, timeout):
        if not self.interpreter.is_file() or not self.worker.is_file():
            raise ValueError('标定计算环境未安装，照片已保存在本机')
        try:
            result = subprocess.run([str(self.interpreter), str(self.worker), *map(str, args)],
                                    capture_output=True, text=True, timeout=timeout, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ValueError('标定程序无法运行：' + str(exc)) from exc
        try:
            payload = json.loads(result.stdout.strip())
        except json.JSONDecodeError as exc:
            raise ValueError('标定程序没有返回有效结果：' + result.stderr[-500:]) from exc
        if result.returncode or not payload.get('ok'):
            raise ValueError(payload.get('error', '标定程序运行失败'))
        return payload

    def add_photo(self, session_id, image, source='camera'):
        if not isinstance(image, bytes) or not 0 < len(image) <= 10_000_000:
            raise ValueError('照片大小无效（最多 10 MB）')
        if image.startswith(b'\xff\xd8\xff'):
            extension = 'jpg'
        elif image.startswith(b'\x89PNG\r\n\x1a\n'):
            extension = 'png'
        else:
            raise ValueError('请选择 JPEG 或 PNG 图片')
        with self.lock:
            data = self._read(session_id)
            if len(data['photos']) >= 200:
                raise ValueError('这组照片已达到 200 张，请新建采集组')
            name = 'photo-%04d.%s' % (len(data['photos']) + 1, extension)
            path = self.directory(session_id) / name
            with path.open('xb') as f:
                f.write(image)
            board = data['board']
            try:
                check = self._worker(['inspect', path, '--cols', board['cols'], '--rows', board['rows']], 15)
                detected = check['detected']
                dimensions = [check['width'], check['height']]
            except ValueError:
                detected, dimensions = None, None
            saved_at = self.now()
            photo = {'file': name, 'captured_at': saved_at if source == 'camera' else None,
                     'saved_at': saved_at, 'source': source,
                     'enabled': True, 'detected': detected, 'dimensions': dimensions}
            data['photos'].append(photo)
            data['revision'] += 1
            self._write(data)
            return self._detail(data)

    def capture(self, session_id, port):
        # Check freshness before saving the current frame, even when the bridge is reachable.
        started = time.monotonic()
        try:
            with urlopen('http://127.0.0.1:%d/health' % port, timeout=3) as response:
                health = json.loads(response.read(1_000_001))
            age = health.get('camera_age_s')
            if isinstance(age, bool) or not isinstance(age, (int, float)) or not 0 <= age < 3:
                raise ValueError('相机画面尚未更新，请检查小车连接')
            with urlopen('http://127.0.0.1:%d/snapshot.jpg' % port, timeout=3) as response:
                image = response.read(10_000_001)
            if age + time.monotonic() - started >= 3:
                raise ValueError('相机画面已过期，请重试')
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError('无法读取小车相机画面') from exc
        return self.add_photo(session_id, image)

    def set_enabled(self, session_id, filename, enabled):
        if not isinstance(filename, str) or not PHOTO_RE.fullmatch(filename) or not isinstance(enabled, bool):
            raise ValueError('照片选择无效')
        with self.lock:
            data = self._read(session_id)
            photo = next((p for p in data['photos'] if p['file'] == filename), None)
            if photo is None:
                raise ValueError('找不到照片')
            if photo['enabled'] != enabled:
                photo['enabled'] = enabled
                data['revision'] += 1
                self._write(data)
            return self._detail(data)

    def photo(self, session_id, filename):
        if not isinstance(filename, str) or not PHOTO_RE.fullmatch(filename):
            raise ValueError('照片编号无效')
        with self.lock:
            data = self._read(session_id)
            if not any(p['file'] == filename for p in data['photos']):
                raise ValueError('找不到照片')
            return (self.directory(session_id) / filename).read_bytes(), ('image/png' if filename.endswith('.png') else 'image/jpeg')

    def compute(self, session_id):
        with self.lock:
            data = self._read(session_id)
            directory = self.directory(session_id)
            filename = 'camera-intrinsics-' + datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8] + '.yaml'
            result = self._worker(['compute', directory, directory / filename], 90)
            result['created_at'] = self.now()
            data['results'].append(result)
            self._write(data)
            return self._detail(data)

    def result_file(self, session_id, filename):
        with self.lock:
            data = self._read(session_id)
            if not isinstance(filename, str) or not any(r['file'] == filename for r in data['results']):
                raise ValueError('找不到标定结果')
            return (self.directory(session_id) / filename).read_bytes()

    def export(self, session_id):
        with self.lock:
            data = self._read(session_id)
            directory = self.directory(session_id)
            names = ['session.json'] + [p['file'] for p in data['photos']] + [r['file'] for r in data['results']]
            if sum((directory / name).stat().st_size for name in names) > 100_000_000:
                raise ValueError('照片总量超过 100 MB，请直接从本机文件夹复制')
            buffer = io.BytesIO()
            with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as archive:
                for name in names:
                    archive.write(directory / name, arcname=name)
            return buffer.getvalue()
