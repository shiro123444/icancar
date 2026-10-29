"""Task 2 event-state prototype. Offline only; no ROS or actuator output.

Inputs must come from verified perception/odometry/line-crossing components.
Timestamps must be monotonic seconds in one clock domain.
"""
import math

class Mission:
    def __init__(self, stop_seconds=3.0, stationary_speed=0.02, feedback_timeout=0.5):
        for v in (stop_seconds,stationary_speed,feedback_timeout):
            if not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or v<=0:
                raise ValueError('参数必须为正有限数')
        if stop_seconds < 3.0: raise ValueError('停车时间不得少于 3 秒')
        self.stop_seconds=float(stop_seconds);self.stationary_speed=float(stationary_speed);self.feedback_timeout=float(feedback_timeout)
        self.phase='idle';self.laps=0;self.started_at=None;self.finished_at=None
        self.last_time=None;self.gate_armed=False;self.crosswalk_armed=True
        self.stopped_since=None;self.last_feedback=None;self.blocked=False
        self.yields_completed=0
    def _expire(self, now):
        if self.phase=='yield' and self.last_feedback is not None and now-self.last_feedback>self.feedback_timeout:
            self.stopped_since=None;self.last_feedback=None
    def process(self,event,now,value=None):
        if not isinstance(now,(int,float)) or isinstance(now,bool) or not math.isfinite(now): raise ValueError('事件时间必须是有限数字')
        now=float(now)
        if self.last_time is not None and now < self.last_time: raise ValueError('事件时间必须单调递增')
        allowed={'start','crosswalk','crosswalk_clear','motion','gate_clear','gate_enter','obstacle','obstacle_clear','tick'}
        if event not in allowed: raise ValueError('未知事件')
        if event=='motion' and (not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value)):
            raise ValueError('运动反馈必须是有限速度')
        self._expire(now);self.last_time=now
        if event=='start':
            if self.phase!='idle':raise ValueError('当前任务已开始')
            self.phase='running';self.started_at=now
        elif event=='crosswalk' and self.phase=='running' and self.crosswalk_armed:
            self.phase='yield';self.crosswalk_armed=False;self.stopped_since=None;self.last_feedback=None
        elif event=='motion' and self.phase=='yield':
            self.last_feedback=now
            if abs(float(value)) > self.stationary_speed:self.stopped_since=None
            else:
                if self.stopped_since is None:self.stopped_since=now
                if now-self.stopped_since>=self.stop_seconds:
                    self.phase='running';self.stopped_since=None;self.last_feedback=None;self.yields_completed+=1
        elif event=='crosswalk_clear' and self.phase=='running':self.crosswalk_armed=True
        elif event=='obstacle' and self.phase in ('running','yield'):self.blocked=True
        elif event=='obstacle_clear' and self.phase in ('running','yield'):self.blocked=False
        elif event=='gate_clear' and self.phase in ('running','yield'):self.gate_armed=True
        elif event=='gate_enter' and self.phase in ('running','yield') and self.gate_armed:
            self.gate_armed=False;self.laps+=1
            if self.laps>=2:self.phase='finished';self.finished_at=now
        return self.snapshot(now)
    def snapshot(self,now):
        if self.phase=='yield':action='stop';message='人行道前停车礼让行人'
        elif self.phase=='running' and not self.blocked:action='follow_lane';message=''
        else:action='stop';message=''
        elapsed=(self.finished_at if self.finished_at is not None else now)-self.started_at if self.started_at is not None else 0.0
        return {'phase':self.phase,'laps':self.laps,'action':action,'announcement':message,'blocked':self.blocked,
                'crosswalk_stops_completed':self.yields_completed,
                'stationary_seconds':round(now-self.stopped_since,2) if self.stopped_since is not None else 0.0,
                'elapsed_seconds':round(elapsed,2),'gate_armed':self.gate_armed,'crosswalk_armed':self.crosswalk_armed}
