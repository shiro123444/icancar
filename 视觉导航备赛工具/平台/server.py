#!/usr/bin/env python3
"""Local simulator and API. The simulator has no hardware output."""
import argparse
import base64
import json
import math
import threading
import time
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs
from urllib.request import urlopen
from urllib.error import URLError, HTTPError
from workflow import TrainingJob
from mission.demo import demo as mission_demo
from automation.demo import demo as automatic_demo
from tools.audit_data import audit
from race_record import RaceRecords
from calibration import CalibrationSessions
from automation.map_localization import LocalizationRuntime

ROOT = Path(__file__).resolve().parent
TRACK = json.loads((ROOT / 'data' / 'track.json').read_text(encoding='utf-8'))
class Simulator:
    def __init__(self):
        self.lock = threading.Lock()
        self.log = deque(maxlen=80)
        self.pose = dict(TRACK['sim_start'])
        self.linear = 0.0
        self.angular = 0.0
        self.deadline = 0.0
        self.last = time.monotonic()
        self.estop = False
        self.note('模拟底盘已就绪')
    def note(self, message):
        self.log.appendleft({'time': time.strftime('%H:%M:%S'), 'message': message})
    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.2)
        if self.deadline and now > self.deadline:
            self.linear = self.angular = 0.0
            self.deadline = 0.0
            self.note('遥控信号超时，已停止')
        angle = math.radians(self.pose['heading_deg'])
        self.pose['x'] += math.cos(angle) * self.linear * TRACK['sim_units_per_second'] * dt
        self.pose['y'] += math.sin(angle) * self.linear * TRACK['sim_units_per_second'] * dt
        self.pose['heading_deg'] = (self.pose['heading_deg'] + self.angular * 90 * dt + 180) % 360 - 180
        self.last = now
    def state(self):
        with self.lock:
            self.tick()
            return {'mode': 'simulator', 'pose': self.pose.copy(), 'linear': self.linear, 'angular': self.angular, 'estop': self.estop, 'logs': list(self.log)}
    def command(self, data):
        with self.lock:
            self.tick()
            action = data.get('action')
            if action == 'stop':
                self.linear = self.angular = 0.0; self.deadline = 0.0
            elif action == 'estop':
                self.linear = self.angular = 0.0; self.deadline = 0.0; self.estop = True; self.note('急停已触发')
            elif action == 'reset_estop':
                self.estop = False; self.note('急停已解除')
            elif action == 'reset_pose':
                self.linear = self.angular = 0.0; self.deadline = 0.0; self.pose = dict(TRACK['sim_start']); self.note('模拟位置已复位')
            elif action == 'drive':
                if self.estop: raise ValueError('请先解除急停')
                linear, angular = data.get('linear'), data.get('angular')
                if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 1 for v in (linear, angular)):
                    raise ValueError('控制值必须在 -1 到 1 之间')
                self.linear, self.angular = float(linear), float(angular)
                self.deadline = time.monotonic() + 0.5
            else: raise ValueError('未知操作')

SIM = Simulator()
TRAINING = TrainingJob()
class Handler(SimpleHTTPRequestHandler):
    def handle(self):
        try: super().handle()
        except (BrokenPipeError, ConnectionResetError): pass
    def log_message(self, format, *args):
        if urlparse(self.path).path not in ('/api/state','/api/status','/api/teleop','/api/workflow/status','/api/robot/telemetry','/api/robot/camera','/api/race-record/current'):
            super().log_message(format, *args)
    def __init__(self, *args, **kwargs): super().__init__(*args, directory=str(ROOT / 'web'), **kwargs)
    def reply(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status); self.send_header('Content-Type','application/json; charset=utf-8'); self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(body)
    def file_reply(self, body, content_type, filename=None):
        self.send_response(200); self.send_header('Content-Type',content_type); self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store')
        if filename:self.send_header('Content-Disposition','attachment; filename="%s"' % filename)
        self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        path = urlparse(self.path).path
        if path.startswith('/api/calibration/'):
            if self.headers.get('Host','') not in ('127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)):
                return self.reply({'error':'invalid host'},403)
            try:
                params=parse_qs(urlparse(self.path).query)
                session_id=params.get('id',[''])[0]
                filename=params.get('file',[''])[0]
                if path == '/api/calibration/sessions':return self.reply({'sessions':self.server.calibrations.list()})
                if path == '/api/calibration/session':return self.reply(self.server.calibrations.detail(session_id))
                if path == '/api/calibration/photo':
                    content,kind=self.server.calibrations.photo(session_id,filename)
                    return self.file_reply(content,kind)
                if path == '/api/calibration/result':
                    content=self.server.calibrations.result_file(session_id,filename)
                    return self.file_reply(content,'application/yaml; charset=utf-8',filename)
                if path == '/api/calibration/export':
                    content=self.server.calibrations.export(session_id)
                    return self.file_reply(content,'application/zip',session_id+'.zip')
                return self.reply({'error':'not found'},404)
            except (ValueError, OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
                return self.reply({'error':str(exc)},400)
        if path in ('/api/robot/telemetry', '/api/robot/camera'):
            target = '/health' if path.endswith('telemetry') else '/snapshot.jpg'
            try:
                with urlopen('http://127.0.0.1:%d%s' % (self.server.telemetry_port,target),timeout=3.0) as response:
                    content=response.read(10_000_001)
                    if len(content)>10_000_000: raise ValueError('telemetry response too large')
                    if target == '/health':
                        value=json.loads(content)
                        if getattr(self.server,'localization',None) is not None:value['localization']=self.server.localization.ingest(value)
                        return self.reply(value)
                    self.send_response(200); self.send_header('Content-Type','image/jpeg'); self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(content))); self.end_headers(); self.wfile.write(content); return
            except (BrokenPipeError, ConnectionResetError):
                return
            except (OSError, ValueError, URLError, HTTPError):
                return self.reply({'ok':False,'msg':'等待小车只读桥'},503)
        if path.startswith('/api/race-record/'):
            if self.headers.get('Host','') not in ('127.0.0.1:'+str(self.server.server_port), 'localhost:'+str(self.server.server_port)):
                return self.reply({'error':'invalid host'},403)
            if path == '/api/race-record/current':return self.reply(self.server.records.snapshot())
            if path == '/api/race-record/history':return self.reply({'records':self.server.records.history()})
            if path == '/api/race-record/export':
                result=self.server.records.snapshot()
                if not result['record']:return self.reply({'error':'没有可导出的记录'},404)
                body=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False).encode('utf-8')
                self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Disposition','attachment; filename="task2-record.json"');self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body);return
            return self.reply({'error':'not found'},404)
        if path == '/api/workflow/status':
            if self.headers.get('Host','') not in ('127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)):
                return self.reply({'error':'invalid host'},403)
            return self.reply(TRAINING.status())
        if path == '/api/ping': return self.reply({'ok': True, 'hostname': '视觉导航模拟器', 'version': 1, 'web_port': self.server.server_port})
        if path == '/api/status':
            state = SIM.state()
            return self.reply({'ok':True,'hostname':'视觉导航模拟器','workspace':'本地模拟','version':1,'driver_ready':True,'teleop':True,'teleop_topic':'模拟底盘','map_ready':False,'camera':False,'actions':[],'race':{},'loc':{},'guard':{},'follow':{},'sensors':{},'remote':{},'simulation':True,'sim_pose':state['pose']})
        if path == '/api/teleop/stop':
            SIM.command({'action':'stop'}); return self.reply({'ok':True})
        if path == '/api/mission/demo': return self.reply(mission_demo())
        if path == '/api/automation/demo': return self.reply(automatic_demo())
        if path == '/api/track': return self.reply(TRACK)
        if path == '/api/track-scan':
            p=ROOT/'data/地图扫描/现场局部-20261029.json'
            return self.reply(json.loads(p.read_text()) if p.is_file() else {'yellow_contours':[],'image_size':[1280,960]})
        if path == '/api/localization/last-trial':
            p=ROOT/'data/地图扫描/field-lap3-map-estimate.json'
            return self.reply(json.loads(p.read_text()) if p.is_file() else {'state':'waiting_for_anchor'})
        if path == '/api/state': return self.reply(SIM.state())
        if path.startswith('/api/'): return self.reply({'ok':False,'msg':'模拟模式暂未接入此功能'},404)
        return super().do_GET()
    def do_POST(self):
        path = urlparse(self.path).path
        try:
            origin = self.headers.get('Origin')
            if origin and origin not in ('http://127.0.0.1:'+str(self.server.server_port), 'http://localhost:'+str(self.server.server_port)):
                return self.reply({'error':'invalid origin'},403)
            if path == '/api/teleop/stop':
                SIM.command({'action':'stop'}); return self.reply({'ok':True})
            if path == '/api/teleop':
                q = parse_qs(urlparse(self.path).query)
                pwm = float(q['lin'][0]); angle = float(q['ang'][0])
                if not (1300 <= pwm <= 1700 and 0 <= angle <= 180): raise ValueError('遥控值超出范围')
                SIM.command({'action':'drive','linear': (pwm-1500)/200, 'angular': (angle-90)/90})
                return self.reply({'ok':True})
            if path.startswith('/api/calibration/'):
                if self.headers.get('Host','') not in ('127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)):
                    return self.reply({'error':'invalid host'},403)
                if self.headers.get_content_type()!='application/json':return self.reply({'error':'content type must be application/json'},415)
                size=int(self.headers.get('Content-Length','0'))
                if size<=0 or size>14_000_000:raise ValueError('请求大小无效')
                payload=json.loads(self.rfile.read(size))
                if not isinstance(payload,dict):raise ValueError('请求格式错误')
                manager=self.server.calibrations
                if path == '/api/calibration/create':return self.reply(manager.create(payload.get('cols'),payload.get('rows'),payload.get('square_mm')))
                if path == '/api/calibration/capture':return self.reply(manager.capture(payload.get('id'),self.server.telemetry_port))
                if path == '/api/calibration/import':
                    encoded=payload.get('image')
                    if not isinstance(encoded,str):raise ValueError('请选择照片')
                    return self.reply(manager.add_photo(payload.get('id'),base64.b64decode(encoded,validate=True),'import'))
                if path == '/api/calibration/toggle':return self.reply(manager.set_enabled(payload.get('id'),payload.get('file'),payload.get('enabled')))
                if path == '/api/calibration/compute':return self.reply(manager.compute(payload.get('id')))
                return self.reply({'error':'not found'},404)
            if path == '/api/localization/anchor':
                if self.headers.get('Host','') not in ('127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)):
                    return self.reply({'error':'invalid host'},403)
                if self.headers.get_content_type()!='application/json':return self.reply({'error':'content type must be application/json'},415)
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2048:raise ValueError('请求大小无效')
                payload=json.loads(self.rfile.read(size))
                if not isinstance(payload,dict):raise ValueError('请求格式错误')
                landmark=payload.get('landmark')
                if not isinstance(landmark,int) or isinstance(landmark,bool) or not 1<=landmark<=8:raise ValueError('请选择地标')
                with urlopen('http://127.0.0.1:%d/health'%self.server.telemetry_port,timeout=3) as r:status=json.load(r)
                pose=status.get('pose');age=status.get('odom_age_s')
                if not isinstance(pose,dict) or not isinstance(age,(int,float)) or age>.5:raise ValueError('等待新鲜里程计')
                return self.reply(self.server.localization.seed(landmark,[pose['x'],pose['y'],pose['yaw_rad']],-90 if landmark==1 else None))
            if path.startswith('/api/race-record/'):
                if self.headers.get('Host','') not in ('127.0.0.1:'+str(self.server.server_port),'localhost:'+str(self.server.server_port)):
                    return self.reply({'error':'invalid host'},403)
                if self.headers.get_content_type()!='application/json':return self.reply({'error':'content type must be application/json'},415)
                size=int(self.headers.get('Content-Length','0'))
                if size<=0 or size>16384:raise ValueError('请求大小无效')
                payload=json.loads(self.rfile.read(size))
                if not isinstance(payload,dict):raise ValueError('请求格式错误')
                if path == '/api/race-record/event':
                    if payload.get('action')!='start' and not isinstance(payload.get('id'),str):raise ValueError('请先选择记录')
                    return self.reply(self.server.records.event(payload.get('action'),payload.get('id')))
                if path == '/api/race-record/review':
                    if not isinstance(payload.get('id'),str):raise ValueError('请先选择记录')
                    return self.reply(self.server.records.review(payload))
                if path == '/api/race-record/select':return self.reply(self.server.records.select(payload.get('id'),payload.get('expected_id')))
                return self.reply({'error':'not found'},404)
            if path.startswith('/api/workflow/'):
                host=self.headers.get('Host','')
                if host not in ('127.0.0.1:'+str(self.server.server_port), 'localhost:'+str(self.server.server_port)):
                    return self.reply({'error':'invalid host'},403)
                size=int(self.headers.get('Content-Length','0'))
                if size <= 0 or size > 8192: raise ValueError('请求大小无效')
                payload=json.loads(self.rfile.read(size))
                if not isinstance(payload,dict):raise ValueError('请求格式错误')
                if path == '/api/workflow/audit':
                    csv_path=payload.get('csv','')
                    if not isinstance(csv_path,str) or not Path(csv_path).is_absolute():raise ValueError('请输入 CSV 的绝对路径')
                    return self.reply(audit(Path(csv_path)))
                if path == '/api/workflow/train': return self.reply(TRAINING.start(payload.get('interpreter'),payload.get('script'),payload.get('csv')))
                if path == '/api/workflow/local-train': return self.reply(TRAINING.start_local(payload.get('source'),payload.get('epochs',20)))
                if path == '/api/workflow/stop': return self.reply(TRAINING.stop())
                return self.reply({'error':'not found'},404)
            if path != '/api/command': return self.reply({'ok':False,'msg':'模拟模式暂未接入此功能'},404)
            size = int(self.headers.get('Content-Length','0'))
            if size <= 0 or size > 2048: raise ValueError('请求大小无效')
            data = json.loads(self.rfile.read(size))
            if not isinstance(data, dict): raise ValueError('请求格式错误')
            SIM.command(data)
            return self.reply(SIM.state())
        except (ValueError, TypeError, KeyError, IndexError, OSError, UnicodeError, json.JSONDecodeError) as exc: return self.reply({'error':str(exc)},400)

def main():
    parser = argparse.ArgumentParser(description='视觉导航本地模拟调试平台')
    parser.add_argument('--port',type=int,default=8766)
    parser.add_argument('--telemetry-port',type=int,default=8770,help='本地 ROS 2 只读桥或 SSH 转发端口')
    parser.add_argument('--calibration-python',default=str(ROOT / '.venv-calibration' / 'bin' / 'python'),help='本机标定程序的 Python 解释器')
    args = parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1',args.port), Handler)
    server.telemetry_port = args.telemetry_port
    server.localization = LocalizationRuntime(TRACK)
    server.records = RaceRecords(ROOT / 'data' / '比赛记录')
    server.calibrations = CalibrationSessions(ROOT / 'data' / '相机标定', args.calibration_python, ROOT / 'tools' / 'calibration_worker.py')
    print(f'打开 http://127.0.0.1:{args.port}/  （Ctrl+C 停止）',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.records.close(); TRAINING.close(); server.server_close()
if __name__ == '__main__': main()
