"""Manual task-2 observations and self-test penalties. Never commands a vehicle."""
import copy
import json
import os
import re
import threading
import time
import uuid
from datetime import datetime
from decimal import Decimal, InvalidOperation
from pathlib import Path

PENALTY_FIELDS=('barrier','boundary','intervention','crosswalk1','announcement1','crosswalk2','announcement2','other')
COUNT_FIELDS=('barrier','boundary','intervention')
ID_RE=re.compile(r'^run-[0-9a-f]{32}$')

def seconds(value):
    if value is None or value == '': return None
    if isinstance(value,bool) or not isinstance(value,(str,int,float)): raise ValueError('时间或罚时必须是非负数字')
    try:
        n=Decimal(str(value))
        if not n.is_finite() or n < 0 or n > 10_000_000 or n != n.quantize(Decimal('0.01')):raise ValueError
        return format(n.quantize(Decimal('0.01')),'f')
    except (InvalidOperation,ValueError):raise ValueError('时间或罚时需为非负数字，最多两位小数')

def current_wall():return datetime.now().astimezone().isoformat(timespec='seconds')

class RaceRecords:
    def __init__(self,directory,clock=time.monotonic,wall=current_wall):
        self.directory=Path(directory);self.clock=clock;self.wall=wall
        self.lock=threading.RLock();self.current=None;self.start_clock=None
        self._restore_latest()
    def _files(self):
        if not self.directory.is_dir(): return []
        return sorted((p for p in self.directory.glob('run-*.json') if ID_RE.fullmatch(p.stem)), key=lambda p:p.stat().st_mtime,reverse=True)
    def _restore_latest(self):
        for f in self._files():
            try:
                rec=json.loads(f.read_text(encoding='utf-8'))
                if rec.get('version')!=1 or rec.get('id')!=f.stem:continue
                self.current=rec
                if rec.get('phase')=='running':
                    rec=copy.deepcopy(rec);rec['phase']='interrupted';rec['interrupted_at']=self.wall();self._save(rec)
                break
            except (OSError,ValueError,TypeError,AttributeError):continue
    def _save(self,rec):
        if not ID_RE.fullmatch(rec['id']):raise ValueError('记录编号无效')
        self.directory.mkdir(parents=True,exist_ok=True)
        target=self.directory/(rec['id']+'.json')
        tmp=self.directory/('.'+rec['id']+'-'+uuid.uuid4().hex+'.tmp')
        try:
            with tmp.open('x',encoding='utf-8') as f:
                json.dump(rec,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
            os.replace(tmp,target)
        finally:
            if tmp.exists():tmp.unlink()
        self.current=rec
    def _elapsed(self,now):
        if self.current and self.current['phase']=='running' and self.start_clock is not None:
            return round(max(0.0,now-self.start_clock),2)
        return self.current.get('manual_elapsed_s') if self.current else None
    def snapshot(self):
        with self.lock:
            if not self.current:return {'phase':'idle','record':None,'manual_elapsed_s':None,'penalty_total':None,'effective_time':None}
            rec=copy.deepcopy(self.current);elapsed=self._elapsed(self.clock())
            all_penalties=list(rec['penalties'].values())
            total=sum((Decimal(x) for x in all_penalties),Decimal('0')) if all(x is not None for x in all_penalties) else None
            effective=Decimal(rec['video_time_s'])+total if rec['video_time_s'] is not None and total is not None else None
            return {'phase':rec['phase'],'record':rec,'manual_elapsed_s':elapsed,
                    'penalty_total':format(total,'.2f') if total is not None else None,
                    'effective_time':format(effective,'.2f') if effective is not None else None}
    def event(self,action,expected_id=None):
        with self.lock:
            now=self.clock()
            if expected_id is not None and (not self.current or expected_id!=self.current['id']):
                raise ValueError('当前记录已改变，请刷新后重试')
            if action=='start':
                if self.current and self.current['phase']=='running':raise ValueError('已有计时正在进行')
                rec={'version':1,'id':'run-'+uuid.uuid4().hex,'phase':'running','started_at':self.wall(),'finished_at':None,
                     'last_marked_elapsed_s':0.0,'manual_elapsed_s':None,'lap_times_s':[],
                     'parking':[{'start_s':None,'end_s':None,'duration_s':None},{'start_s':None,'end_s':None,'duration_s':None}],
                     'broadcast':['unknown','unknown'],'video_time_s':None,'video_parking_s':[None,None],
                     'counts':{key:None for key in COUNT_FIELDS},
                     'penalties':{key:None for key in PENALTY_FIELDS},'notes':'',
                     'clock_source':'manual_observation','rules_version':'self-test-vs-notice-unconfirmed'}
                self._save(rec);self.start_clock=now
                return self.snapshot()
            if not self.current or self.current['phase']!='running' or self.start_clock is None:raise ValueError('请先开始计时')
            elapsed=self._elapsed(now);rec=copy.deepcopy(self.current);lap=len(rec['lap_times_s'])
            if action=='park_start':
                if lap>=2 or rec['parking'][lap]['start_s'] is not None:raise ValueError('本圈已记录停车')
                rec['parking'][lap]['start_s']=elapsed
            elif action=='park_end':
                park=rec['parking'][lap]
                if park['start_s'] is None or park['end_s'] is not None:raise ValueError('本圈没有进行中的停车记录')
                park['end_s']=elapsed;park['duration_s']=round(max(0,elapsed-park['start_s']),2)
            elif action=='park_clear':
                if lap>=2:raise ValueError('比赛已结束')
                rec['parking'][lap]={'start_s':None,'end_s':None,'duration_s':None}
            elif action=='lap':
                park=rec['parking'][lap]
                if park['start_s'] is not None and park['end_s'] is None:raise ValueError('请先结束本圈停车记录')
                rec['lap_times_s'].append(elapsed)
                if len(rec['lap_times_s'])==2:
                    rec['phase']='finished';rec['manual_elapsed_s']=elapsed;rec['finished_at']=self.wall()
            elif action=='interrupt':
                rec['phase']='interrupted';rec['manual_elapsed_s']=elapsed;rec['interrupted_at']=self.wall()
            else:raise ValueError('未知记录操作')
            rec['last_marked_elapsed_s']=elapsed;self._save(rec)
            if rec['phase']!='running':self.start_clock=None
            return self.snapshot()
    def review(self,data):
        with self.lock:
            if not self.current:raise ValueError('请先开始或选择一条记录')
            if not isinstance(data,dict):raise ValueError('复核数据格式无效')
            if data.get('id') is not None and data['id']!=self.current['id']:
                raise ValueError('当前记录已改变，请刷新后重试')
            rec=copy.deepcopy(self.current)
            if 'video_time_s' in data:rec['video_time_s']=seconds(data['video_time_s'])
            if 'penalties' in data:
                v=data['penalties']
                if not isinstance(v,dict) or set(v)!=set(PENALTY_FIELDS):raise ValueError('请核对全部罚时项目')
                rec['penalties']={k:seconds(v[k]) for k in PENALTY_FIELDS}
            if 'broadcast' in data:
                v=data['broadcast']
                if not isinstance(v,list) or len(v)!=2 or any(x not in ('unknown','yes','no') for x in v):raise ValueError('播报记录无效')
                rec['broadcast']=v[:]
            if 'video_parking_s' in data:
                v=data['video_parking_s']
                if not isinstance(v,list) or len(v)!=2:raise ValueError('停车时长需分别填写两圈')
                rec['video_parking_s']=[seconds(x) for x in v]
            if 'counts' in data:
                v=data['counts']
                if not isinstance(v,dict) or set(v)!=set(COUNT_FIELDS):raise ValueError('请核对违例次数')
                counts={}
                for key,value in v.items():
                    if value is None or value=='':counts[key]=None;continue
                    if isinstance(value,bool) or not isinstance(value,(str,int)) or not str(value).isdigit() or len(str(value))>7:
                        raise ValueError('违例次数必须是非负整数')
                    counts[key]=int(value)
                rec['counts']=counts
            if 'notes' in data:
                if not isinstance(data['notes'],str) or len(data['notes'])>2000:raise ValueError('备注最多 2000 字')
                rec['notes']=data['notes']
            rec['reviewed_at']=self.wall();self._save(rec);return self.snapshot()
    def history(self):
        with self.lock:
            out=[]
            for p in self._files()[:100]:
                try:
                    rec=json.loads(p.read_text(encoding='utf-8'))
                    if rec.get('id')!=p.stem or rec.get('version')!=1:continue
                    out.append({'id':rec['id'],'phase':rec['phase'],'started_at':rec.get('started_at'),'video_time_s':rec.get('video_time_s')})
                except (OSError,ValueError,TypeError,AttributeError):continue
            return out
    def select(self,run_id,expected_id=None):
        with self.lock:
            if expected_id is not None and (not self.current or expected_id!=self.current['id']):
                raise ValueError('当前记录已改变，请刷新后重试')
            if self.current and self.current['phase']=='running':raise ValueError('计时进行中，不能切换记录')
            if not isinstance(run_id,str) or not ID_RE.fullmatch(run_id):raise ValueError('记录编号无效')
            try:rec=json.loads((self.directory/(run_id+'.json')).read_text(encoding='utf-8'))
            except (OSError,ValueError) as exc:raise ValueError('记录无法读取') from exc
            if rec.get('id')!=run_id or rec.get('version')!=1:raise ValueError('记录格式无效')
            if rec.get('phase')=='running':
                rec['phase']='interrupted';rec['interrupted_at']=self.wall();self._save(rec)
            self.current=rec;self.start_clock=None;return self.snapshot()
    def close(self):
        with self.lock:
            if self.current and self.current['phase']=='running':self.event('interrupt')
