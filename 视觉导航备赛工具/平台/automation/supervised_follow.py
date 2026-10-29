"""Operator-supervised visual trials, separate from calibrated race operation."""
import math
from automation.race import Edge, number
from automation.steering_candidate import SteeringCandidate, fresh_frame
from automation.model_crosswalk import ModelCrosswalk
from automation.visual_route import RouteGapPlanner


def view_quality(image):
    """Flag a clearly obscured view; colour and texture are diagnostic evidence."""
    import cv2
    import numpy as np
    image=np.asarray(image)
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3 or min(image.shape[:2])<32:
        raise ValueError('invalid view image')
    roi=image[int(image.shape[0]*.35):int(image.shape[0]*.98)]
    hsv=cv2.cvtColor(roi,cv2.COLOR_RGB2HSV)
    yellow=float(((hsv[:,:,0]>=18)&(hsv[:,:,0]<=42)&(hsv[:,:,1]>=70)&(hsv[:,:,2]>=70)).mean())
    gray=cv2.cvtColor(roi,cv2.COLOR_RGB2GRAY)
    texture=float(cv2.Laplacian(cv2.GaussianBlur(gray,(5,5),1),cv2.CV_64F).var())
    return dict(visible=yellow>=.003 or texture>=5,yellow_fraction=round(yellow,5),texture=round(texture,3))


class FollowTrial:
    def __init__(self, model_config, speed=.2, max_seconds=120, min_lap_distance=8,
                 stop_after_yield=False, finish_on_lap=True, visual_route_assist=False):
        if not all(number(x) and x>0 for x in (speed,max_seconds,min_lap_distance)) or speed>.2 or max_seconds>180:
            raise ValueError('supervised trial limits invalid')
        self.speed,self.max_seconds,self.min_lap_distance=speed,max_seconds,min_lap_distance
        self.stop_after_yield,self.finish_on_lap=stop_after_yield,finish_on_lap
        self.candidate=SteeringCandidate(speed,.65,.45,1,True,.45,.25,3)
        self.model=ModelCrosswalk(model_config)
        self.line=Edge(3);self.stripes=Edge(3);self.model_edge=Edge(2)
        self.phase,self.reason='waiting','waiting_for_real_inputs'
        self.image_at=self.image_stamp=self.odom_at=self.odom_stamp=None
        self.velocity=self.pose=self.origin=None
        self.started=self.ready_since=self.stationary_since=None
        self.last_tick=None;self.command=(0.,0.,0.)
        self.distance=0.;self.stops=self.laps=0
        self.yield_locked=False;self.yield_distance=0.;self.announcement_id=0;self.announcement_ack=False
        self.blue_frames=self.obscured_frames=0;self.events=[]
        self.peak_linear=self.peak_angular=0.
        self.input_hold_at=None
        self.visual_route_assist=visual_route_assist is True
        self.route_planner=RouteGapPlanner();self.lane_geometry=None;self.lane_geometry_at=None;self.route_proposal=None

    def vision(self,observation,now):
        self.lane_geometry=observation;self.lane_geometry_at=now

    def event(self,kind,now,**values):self.events.append(dict(type=kind,at=now,**values))
    def fail(self,reason,now):
        if self.phase not in ('fault','done'):
            self.phase,self.reason='fault',reason;self.command=(0.,0.,0.);self.event('fault',now,reason=reason)
    def stop(self,now,reason='operator_stop'):
        if self.phase not in ('done','fault'):
            self.phase,self.reason='done',reason;self.command=(0.,0.,0.);self.event('stop',now,reason=reason)

    def steering(self,value,now,ros_now):self.candidate.update(value,now,ros_now)

    def image(self,now,stamp,ros_now,line,crosswalk,blue_fraction,visible=True):
        if not fresh_frame(stamp,ros_now,.45) or (self.image_stamp is not None and stamp<=self.image_stamp):return
        if not isinstance(line,bool) or not isinstance(crosswalk,bool) or not number(blue_fraction) or not 0<=blue_fraction<=1:
            self.fail('invalid_visual_observation',now);return
        self.image_at,self.image_stamp=now,stamp
        self.obscured_frames=0 if visible else self.obscured_frames+1
        if self.obscured_frames>=3 and self.phase in ('follow','yield'):self.fail('camera_view_obscured',now)
        line_edge=self.line.observe(line)
        stripe_edge=self.stripes.observe(crosswalk)
        self.blue_frames=self.blue_frames+1 if blue_fraction>.35 else 0
        if self.blue_frames>=3 and self.phase in ('follow','yield'):
            self.fail('blue_surface_in_near_path',now)
        if stripe_edge:self.crosswalk_trigger(now,'camera_stripes')
        if (line_edge and self.phase=='follow' and self.finish_on_lap and self.distance>=self.min_lap_distance
                and self.stops>=1 and self.origin is not None and self.pose is not None):
            x,y,yaw=self.pose;ox,oy,oyaw=self.origin
            heading=abs(math.atan2(math.sin(yaw-oyaw),math.cos(yaw-oyaw)))
            if math.hypot(x-ox,y-oy)<=.7 and heading<=.65:
                self.laps=1;self.stop(now,'one_lap_reference_return');self.event('lap',now,lap=1)
            else:self.event('line_outside_return_area',now,distance=self.distance,pose=self.pose)
        if (self.yield_locked and self.phase=='follow' and self.distance-self.yield_distance>.7
                and self.stripes.state is False and self.model_edge.state is False):self.yield_locked=False

    def observation(self,value,now,ros_now):
        detected=self.model.update(value,now,ros_now)
        if detected is not None and self.model_edge.observe(detected):self.crosswalk_trigger(now,'openvino_model')

    def crosswalk_trigger(self,now,source):
        if self.phase=='follow' and not self.yield_locked:
            self.phase,self.reason='yield','crosswalk'
            self.yield_locked=True;self.yield_distance=self.distance
            self.stationary_since=None;self.announcement_id+=1;self.announcement_ack=False
            self.command=(0.,0.,0.);self.event('crosswalk_stop',now,source=source,id=self.announcement_id,distance=self.distance)

    def ack(self,identifier,success,now):
        if self.phase=='yield' and identifier==self.announcement_id and success is True:
            self.announcement_ack=True;self.event('speech_command_ack',now,content_verified=False)

    def odometry(self,now,stamp,ros_now,vx,vy,w,x,y,yaw):
        if not all(number(v) for v in (vx,vy,w,x,y,yaw)) or not fresh_frame(stamp,ros_now,.4):return
        if self.odom_stamp is not None and stamp<=self.odom_stamp:return
        dt=0 if self.odom_at is None else now-self.odom_at
        if self.started is not None and 0<dt<=.25:self.distance+=math.hypot(vx,vy)*dt
        self.odom_at,self.odom_stamp,self.velocity,self.pose=now,stamp,(vx,vy,w),(x,y,yaw)
        self.peak_linear=max(self.peak_linear,math.hypot(vx,vy));self.peak_angular=max(self.peak_angular,abs(w))
        if math.hypot(vx,vy)>.35 or abs(w)>1.3:
            if self.phase!='fault':self.event('wheel_feedback_limit_sample',now,measured_velocity=self.velocity,pose=self.pose)
            self.fail('wheel_feedback_limit',now)
        if self.phase=='yield':
            if self.stationary():
                if self.stationary_since is None:self.stationary_since=now
            else:self.stationary_since=None

    def stationary(self):
        return self.velocity is not None and math.hypot(*self.velocity[:2])<=.012 and abs(self.velocity[2])<=.06

    def inputs_ready(self,now,ros_now):
        return (self.image_at is not None and 0<=now-self.image_at<=.45
                and fresh_frame(self.image_stamp,ros_now,.45)
                and self.odom_at is not None and 0<=now-self.odom_at<=.3
                and fresh_frame(self.odom_stamp,ros_now,.4)
                and self.model.available(now,ros_now)
                and self.candidate.output(now,ros_now) not in (None,(0.,0.,0.)))

    def tick(self,now,ros_now):
        dt=.05 if self.last_tick is None else max(0.,min(.1,now-self.last_tick));self.last_tick=now
        if self.phase=='waiting':
            if self.inputs_ready(now,ros_now) and self.stationary() and self.blue_frames<3 and self.obscured_frames<3:
                if self.ready_since is None:self.ready_since=now
                if now-self.ready_since>=.5:
                    self.phase,self.reason,self.started='follow','supervised_follow',now
                    self.origin=self.pose;self.distance=0.;self.event('start',now,origin=self.origin)
            else:self.ready_since=None
            self.command=(0.,0.,0.);return self.command
        if self.phase in ('done','fault'):self.command=(0.,0.,0.);return self.command
        if not self.inputs_ready(now,ros_now):
            if self.input_hold_at is None:
                self.input_hold_at=now;self.event('input_hold',now)
            self.command=(0.,0.,0.)
            ages=[now-t for t in (self.image_at,self.odom_at,self.candidate.received_at,self.model.received_at) if t is not None]
            if now-self.input_hold_at>=.6 or ages and max(ages)>=.9:
                self.fail('camera_model_or_odometry_stale',now)
            return self.command
        if self.input_hold_at is not None:
            self.event('input_resume',now,hold_seconds=now-self.input_hold_at);self.input_hold_at=None
        if now-self.started>=self.max_seconds:
            self.stop(now,'trial_time_limit');return self.command
        if self.distance>22:
            self.fail('trial_distance_limit',now);return self.command
        if self.phase=='yield':
            self.command=(0.,0.,0.)
            if self.stationary_since is not None and now-self.stationary_since>=3 and self.announcement_ack:
                self.stops+=1;self.event('crosswalk_stop_complete',now,stationary_seconds=now-self.stationary_since)
                if self.stop_after_yield:self.stop(now,'crosswalk_test_complete')
                else:self.phase,self.reason='follow','supervised_follow'
            return self.command
        raw=self.candidate.output(now,ros_now)
        angular=raw[2]
        speed_cap=self.speed
        if self.lane_geometry is not None and self.lane_geometry_at is not None and 0<=now-self.lane_geometry_at<=.45:
            self.route_proposal=self.route_planner.propose(self.lane_geometry,self.distance,angular)
            if self.visual_route_assist:
                if self.route_proposal.get('steering') is None:
                    self.command=(0.,0.,0.);return self.command
                angular=self.route_proposal['steering'];speed_cap=min(speed_cap,self.route_proposal['speed_cap'])
        speed=min(self.speed,max(.09,self.speed/(1+1.5*abs(angular))))
        speed=min(speed,speed_cap)
        angular=max(self.command[2]-1.5*dt,min(self.command[2]+1.5*dt,angular))
        speed=min(speed,self.command[0]+.4*dt)
        self.command=(speed,0.,angular)
        return self.command

    def status(self,now):
        return dict(source='supervised_visual_test',active=self.phase in ('follow','yield'),phase=self.phase,
                    reason=self.reason,supervised=True,calibrated=False,laps=self.laps,crosswalk_stops=self.stops,
                    distance_m=round(self.distance,3),elapsed_seconds=0 if self.started is None else round(now-self.started,2),
                    frame_age_s=None if self.image_at is None else round(now-self.image_at,3),
                    steering_age_s=None if self.candidate.received_at is None else round(now-self.candidate.received_at,3),
                    model_age_s=None if self.model.received_at is None else round(now-self.model.received_at,3),
                    feedback_age_s=None if self.odom_at is None else round(now-self.odom_at,3),
                    command=self.command,measured_velocity=self.velocity,pose=self.pose,
                    origin=self.origin,start_line=self.line.state,
                    crosswalk=self.stripes.state is True or self.model_edge.state is True,
                    lane_geometry=None if self.lane_geometry is None else {k:v for k,v in self.lane_geometry.items() if k!='rows'},
                    route_proposal=self.route_proposal,visual_route_active=self.visual_route_assist,
                    peak_linear_feedback=round(self.peak_linear,4),peak_angular_feedback=round(self.peak_angular,4),
                    announcement_ack=self.announcement_ack,announcement_content_verified=False)
