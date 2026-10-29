"""Relative wheel pose plus soft matching to a directed vector route.

Metric scale is an estimate until registered. Ambiguous junctions and reverse
headings are reported, not silently forced onto the expected route branch.
"""
import math
import threading
import time


def angle(x):return math.atan2(math.sin(x),math.cos(x))


class RouteMap:
    def __init__(self,track):
        self.track=track;n=track['navigation'];c=track['coordinate_space'];scale=n['scale_estimate']
        self.sx=scale['width_m']/c['width'];self.sy=scale['height_m']/c['height']
        self.calibrated=scale.get('calibrated') is True
        self.points=[(p[0]*self.sx,p[1]*self.sy) for p in n['points']]
        self.labels=n['point_sections'];self.names={s['id']:s['label'] for s in n['sections']}
        self.s=[];self.headings=[];total=0.
        for i,p in enumerate(self.points):
            q=self.points[(i+1)%len(self.points)];self.s.append(total)
            total+=math.dist(p,q);self.headings.append(math.atan2(q[1]-p[1],q[0]-p[0]))
        self.length=total
    def delta_s(self,a,b):return (a-b+self.length/2)%self.length-self.length/2
    def match(self,x,y,heading,prior=None):
        candidates=[]
        for i,(px,py) in enumerate(self.points):
            d=math.hypot(x-px,y-py);e=angle(heading-self.headings[i])
            orientation=min(abs(e),abs(angle(e-math.pi)))
            score=(d/.35)**2+(orientation/.7)**2
            temporal=0 if prior is None else (self.delta_s(self.s[i],prior)/.65)**2
            candidates.append((score,score+temporal,i,d,e))
        spatial=min(candidates,key=lambda c:c[0]);ordered=min(candidates,key=lambda c:c[1])
        return spatial,ordered


class MapLocalizer:
    def __init__(self,track):
        self.map=RouteMap(track);self.anchor=None;self.origin=None;self.last=None
        self.progress=None;self.fused=None;self.session=None;self.path=[]
        self.last_landmark=None
    def reset(self):
        self.anchor=self.origin=self.last=self.progress=self.fused=self.session=None
        self.path=[];self.last_landmark=None
    def seed(self,landmark,odom,heading_deg=None,session=None):
        choices={p['id']:(p['x'],p['y']) for p in self.map.track['landmarks']}
        if (not isinstance(landmark,int) or isinstance(landmark,bool) or landmark not in choices
                or len(odom)!=3 or not all(math.isfinite(x) for x in odom)
                or heading_deg is not None and not math.isfinite(heading_deg)):raise ValueError('invalid map anchor')
        u,v=choices[landmark]
        if landmark==1:u,v=self.map.track['sim_start']['x'],self.map.track['sim_start']['y']
        if heading_deg is None:
            p=min(range(len(self.map.points)),key=lambda i:math.dist(self.map.points[i],(u*self.map.sx,v*self.map.sy)))
            heading=self.map.headings[p]
        else:heading=math.radians(heading_deg)
        self.anchor=(u*self.map.sx,v*self.map.sy,heading);self.origin=tuple(odom);self.last=None
        self.fused=self.anchor[:2];self.progress=None;self.path=[];self.session=session
        self.last_landmark=None
    def update(self,odom,now,lane=None,landmark=None):
        if self.anchor is None:return dict(state='waiting_for_anchor',metric_calibrated=self.map.calibrated)
        if len(odom)!=3 or not all(math.isfinite(x) for x in (*odom,now)):raise ValueError('invalid odometry')
        if self.last is not None:
            dt=now-self.last[0];distance=math.dist(odom[:2],self.last[1][:2])
            heading_jump=abs(angle(odom[2]-self.last[1][2]))
            if dt<0 or dt==0 and (distance>1e-6 or heading_jump>1e-6) or (0<dt<2 and (distance>max(.25,dt*1.2) or heading_jump>max(.6,dt*3))):
                self.anchor=None;return dict(state='anchor_required',reason='odometry_reset_or_jump',metric_calibrated=self.map.calibrated)
        else:dt=None
        ax,ay,ah=self.anchor;ox,oy,oh=self.origin;x,y,yaw=odom
        alignment=ah+oh;dx,dy=x-ox,y-oy
        px=ax+math.cos(alignment)*dx+math.sin(alignment)*dy
        py=ay+math.sin(alignment)*dx-math.cos(alignment)*dy
        heading=angle(ah-angle(yaw-oh))
        prior=self.progress if dt is not None and dt<2 else None
        spatial,ordered=self.map.match(px,py,heading,prior)
        _,_,i,residual,heading_error=ordered
        conflict=prior is not None and abs(self.map.delta_s(self.map.s[spatial[2]],prior))>1.2 and spatial[0]+1<ordered[0]
        forward=math.cos(heading_error)
        direction='forward' if forward>.4 else 'reverse' if forward<-.4 else 'uncertain'
        confidence=max(.05,min(.9,math.exp(-residual**2/.35**2)))
        if conflict:confidence*=.3
        # Route projection is a soft position proposal. Preserve raw wheel pose.
        rx,ry=self.map.points[i]
        weight=.2 if not conflict and residual<.45 else 0.
        if isinstance(lane,dict) and lane.get('valid') and lane.get('confidence',0)>.5:weight=min(.45,weight+.15)
        fx=(1-weight)*px+weight*rx;fy=(1-weight)*py+weight*ry
        if isinstance(lane,dict) and lane.get('valid') and isinstance(lane.get('lateral_error'),(int,float)) and not conflict:
            normal=(-math.sin(self.map.headings[i]),math.cos(self.map.headings[i]))
            current=(px-rx)*normal[0]+(py-ry)*normal[1]
            correction=max(-.1,min(.1,lane['lateral_error']*.52-current))*.3
            fx+=correction*normal[0];fy+=correction*normal[1]
        if landmark=='crosswalk' and not conflict:
            target=next(p for p in self.map.track['landmarks'] if p['id']==2)
            # Presence constrains the road section; no uncalibrated depth snap.
            if math.hypot(px-target['x']*self.map.sx,py-target['y']*self.map.sy)<1.5:confidence=min(.95,confidence+.1)
            if self.last_landmark!='crosswalk' and abs(angle(heading+math.pi/2))<.8:
                zebra=self.map.track['crosswalk'];landmark_y=(zebra['y']+zebra['stripe_height'])*self.map.sy+.45
                # Weak longitudinal cue: the viewing distance is still approximate.
                fy+=max(-.2,min(.2,landmark_y-fy))*.2
        self.last_landmark=landmark
        self.progress=self.map.s[i];self.fused=(fx,fy);self.last=(now,tuple(odom))
        self.path.append([fx/self.map.sx,fy/self.map.sy]);self.path=self.path[-1500:]
        convert=lambda a,b:dict(x=round(a/self.map.sx,2),y=round(b/self.map.sy,2),heading_deg=round(math.degrees(heading),1))
        return dict(state='tracking',metric_calibrated=self.map.calibrated,confidence=round(confidence,3),
            odometry=convert(px,py),fused=convert(fx,fy),section=self.map.labels[i],section_label=self.map.names[self.map.labels[i]],
            direction=direction,branch_conflict=conflict,route_progress_m=round(self.progress,3),
            lateral_distance_m=round(residual,3),route_length_m_estimate=round(self.map.length,2),path=self.path[-300:])


class LocalizationRuntime:
    """Read-only telemetry consumer; seeding changes the map frame, never the car."""
    def __init__(self,track,clock=time.monotonic):
        self.lock=threading.Lock();self.clock=clock;self.localizer=MapLocalizer(track)
        self.session=None;self.seeded_session=None;self.result=dict(state='waiting_for_anchor',metric_calibrated=False)
    def seed(self,landmark,odom,heading_deg=None):
        with self.lock:
            self.localizer.seed(landmark,odom,heading_deg)
            self.result=self.localizer.update(odom,self.clock())
            return self.result
    def ingest(self,telemetry):
        with self.lock:
            control=telemetry.get('control');age=telemetry.get('control_age_s')
            if isinstance(control,dict) and isinstance(age,(int,float)) and 0<=age<1:
                session=control.get('session')
                if session and session!=self.session:
                    self.localizer.reset();self.session=session;self.seeded_session=None
                    self.result=dict(state='waiting_for_anchor',metric_calibrated=self.localizer.map.calibrated)
                if session and self.localizer.anchor is None and self.seeded_session!=session and control.get('start_anchor')==1 and control.get('origin'):
                    self.localizer.seed(1,control['origin'],-90,session);self.seeded_session=session
                pose=control.get('pose')
                if isinstance(pose,list) and len(pose)==3:
                    self.result=self.localizer.update(pose,self.clock(),lane=control.get('lane_geometry'),landmark='crosswalk' if control.get('crosswalk') else None)
                    return dict(self.result,age_s=age)
            pose=telemetry.get('pose');age=telemetry.get('odom_age_s')
            if self.localizer.anchor is not None and isinstance(pose,dict) and isinstance(age,(int,float)) and 0<=age<.5:
                self.result=self.localizer.update([pose['x'],pose['y'],pose['yaw_rad']],self.clock())
                return dict(self.result,age_s=age)
            return dict(self.result,state='last_known' if self.result.get('fused') else 'waiting_for_anchor')
