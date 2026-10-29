"""Deterministic, local-only dry run of task-2 event logic."""
from .task2 import Mission

def demo():
    m=Mission();timeline=[]
    def event(label,kind,t,value=None):
        s=m.process(kind,t,value)
        if label: timeline.append({'t':round(t,2),'event':label,'phase':s['phase'],'laps':s['laps'],'action':s['action'],'announcement':s['announcement']})
    event('开始','start',0)
    event('离开起点线','gate_clear',0.1)
    event('识别人行道','crosswalk',1)
    for i in range(14):event('确认停车 ≥ 3 秒' if i==13 else None,'motion',1.1+i*.25,0)
    event('驶离人行道','crosswalk_clear',4.4)
    event('检测到障碍','obstacle',5)
    event('障碍处理完成','obstacle_clear',6)
    event('完成第 1 圈','gate_enter',8)
    event('离开起点线','gate_clear',8.1)
    event('识别人行道','crosswalk',9)
    for i in range(14):event('确认停车 ≥ 3 秒' if i==13 else None,'motion',9.1+i*.25,0)
    event('驶离人行道','crosswalk_clear',12.4)
    event('完成第 2 圈','gate_enter',16)
    return {'mode':'offline-demo','timeline':timeline,'result':m.snapshot(16)}
