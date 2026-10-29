#!/usr/bin/env python3
"""Bounded ROS 2 visual experiment with an operator at the power switch."""
import argparse
import io
import json
import math
import os
import sys
import time
import uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from PIL import Image
from automation.detector import detect,validate_config
from automation.supervised_follow import FollowTrial,view_quality
from automation.visual_route import observe_lane
from automation.realtime import EventWriter,LatestWorker
from automation.steering_candidate import fresh_frame


def main():
    p=argparse.ArgumentParser(description='现场看护的 ROS 2 循迹、人行道停车和一圈实验')
    p.add_argument('--config',required=True);p.add_argument('--model-config',required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=120);p.add_argument('--speed',type=float,default=.2)
    p.add_argument('--stop-after-crosswalk',action='store_true')
    p.add_argument('--capture-interval',type=float,default=.5)
    p.add_argument('--start-landmark',type=int,choices=(1,))
    p.add_argument('--visual-route-assist',action='store_true')
    p.add_argument('--operator-at-power-switch',action='store_true');p.add_argument('--run-motion-test',action='store_true')
    p.add_argument('--stop-file',type=Path,required=True)
    args=p.parse_args()
    if (Path(__file__).resolve().parents[1]/'runtime/OPERATOR_HOLD').exists():
        p.error('车辆处于操作员暂停状态')
    if not args.operator_at_power_switch or not args.run_motion_test:p.error('需要现场看护并明确启用实验')
    if args.output_dir.exists():p.error('输出目录已存在')
    if args.stop_file.exists():p.error('停止标记已经存在')
    if not math.isfinite(args.capture_interval) or not .2<=args.capture_interval<=2:p.error('采集间隔须为 0.2–2 秒')
    config=validate_config(json.loads(Path(args.config).read_text()))
    trial=FollowTrial(json.loads(Path(args.model_config).read_text()),args.speed,args.seconds,
                      stop_after_yield=args.stop_after_crosswalk,visual_route_assist=args.visual_route_assist)
    import cv2
    cv2.setNumThreads(1)
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    args.output_dir.mkdir(parents=True)
    (args.output_dir/'frames').mkdir()
    session=uuid.uuid4().hex
    event_path=args.output_dir/'events.jsonl'
    def write_event(value):
        if '_frame_bytes' in value:
            (args.output_dir/value['image_path']).write_bytes(value['_frame_bytes'])
            metadata={k:v for k,v in value.items() if k!='_frame_bytes'}
            destination='snapshots.jsonl' if value.get('type')=='snapshot' else 'frames.jsonl'
            with (args.output_dir/destination).open('a') as out:out.write(json.dumps(metadata,ensure_ascii=False,allow_nan=False)+'\n')
            return
        with event_path.open('a') as out:out.write(json.dumps(value,ensure_ascii=False,allow_nan=False)+'\n')
    writer=EventWriter(write_event)
    rclpy.init()
    class Test(Node):
        def __init__(self):
            super().__init__('visual_race_supervised_track_test')
            self.command_pub=self.create_publisher(TwistStamped,'/visual_race/supervised/cmd_vel',1)
            self.status_pub=self.create_publisher(String,'/visual_race/control_diagnostics',1)
            self.speech=self.create_publisher(String,'/visual_race/speech',10)
            self.create_subscription(CompressedImage,'/camera/rgb/image_raw/compressed',self.camera,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/raw_odom',self.odom,qos_profile_sensor_data)
            self.create_subscription(String,'/visual_race/steering_preview',self.steering,1)
            self.create_subscription(String,'/visual_race/model_detections',self.model,1)
            self.create_subscription(String,'/visual_race/speech_ack',self.ack,10)
            self.create_service(Trigger,'/visual_race/supervised/stop',self.stop)
            self.started=time.monotonic();self.ended=None;self.stopped_since=None;self.last_speech=0.
            self.latest_image=None;self.latest_image_stamp=0;self.done=False;self.last_status=0.;self.command_count=0
            self.view_quality=None
            self.last_capture=-float('inf');self.latest_steering=None
            self.last_camera_submit=0
            self.saved_snapshots=set()
            self.perception_worker=LatestWorker(self.process_image,10)
            self.perception_worker.start()
            self.create_timer(.02,self.image_result,clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        def stamp(self,msg):return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        def json_value(self,msg):
            if len(msg.data)>150000:raise ValueError('oversized model message')
            return json.loads(msg.data,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('invalid JSON')))
        def camera(self,msg):
            stamp=self.stamp(msg)
            if stamp<=self.last_camera_submit or not fresh_frame(stamp,self.get_clock().now().nanoseconds,.45):return
            if not 0<len(msg.data)<=10000000:
                trial.fail('invalid_camera_bytes',time.monotonic());return
            self.last_camera_submit=stamp
            self.perception_worker.submit((stamp,bytes(msg.data)))
        @staticmethod
        def process_image(frame):
            stamp,data=frame;started=time.monotonic()
            with Image.open(io.BytesIO(data)) as im:
                if im.width*im.height>16000000:raise ValueError('oversized image')
                rgb=np.asarray(im.convert('RGB'))
            d=detect(rgb,config);h,w=rgb.shape[:2];quality=view_quality(rgb);lane=observe_lane(rgb)
            hsv=cv2.cvtColor(rgb[int(h*.6):int(h*.9),int(w*.4):int(w*.6)],cv2.COLOR_RGB2HSV)
            blue=float(((hsv[:,:,0]>=90)&(hsv[:,:,0]<=135)&(hsv[:,:,1]>=60)&(hsv[:,:,2]>=35)).mean())
            return dict(stamp=stamp,data=data,detections=d,quality=quality,lane=lane,blue=blue,
                        processing_ms=(time.monotonic()-started)*1000)
        def image_result(self):
            result=self.perception_worker.take()
            if result is None:return
            if not result['ok']:
                trial.fail('image_decode_or_detection:'+result['error'],time.monotonic());return
            value=result['value'];stamp=value['stamp']
            if not fresh_frame(stamp,self.get_clock().now().nanoseconds,.45):return
            try:
                d=value['detections'];self.view_quality=value['quality']
                trial.vision(value['lane'],time.monotonic())
                trial.image(time.monotonic(),stamp,self.get_clock().now().nanoseconds,
                            d['start_line']['detected'],d['crosswalk']['detected'],value['blue'],self.view_quality['visible'])
                self.latest_image=value['data'];self.latest_image_stamp=stamp
                now=time.monotonic()
                if trial.phase in ('follow','yield') and now-self.last_capture>=args.capture_interval:
                    self.last_capture=now
                    if not writer.submit(dict(type='frame',at=now,image_path='frames/'+str(stamp)+'.jpg',
                            image_stamp_ns=stamp,odom_stamp_ns=trial.odom_stamp,odom_pose=trial.pose,
                            actual_velocity=trial.velocity,command=trial.command,steering=self.latest_steering,
                            phase=trial.phase,distance_m=trial.distance,view_quality=self.view_quality,
                            lane_geometry=value['lane'],processing_ms=value['processing_ms'],_frame_bytes=value['data'])):
                        trial.fail('frame_capture_queue_failure',now)
            except Exception as e:trial.fail('image_decode_or_detection:'+str(e),time.monotonic())
        def steering(self,msg):
            try:value=self.json_value(msg)
            except (ValueError,TypeError):value=None
            self.latest_steering=value
            trial.steering(value,time.monotonic(),self.get_clock().now().nanoseconds)
        def model(self,msg):
            try:value=self.json_value(msg)
            except (ValueError,TypeError):value=None
            trial.observation(value,time.monotonic(),self.get_clock().now().nanoseconds)
        def odom(self,msg):
            v=msg.twist.twist;p=msg.pose.pose.position;q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            trial.odometry(time.monotonic(),self.stamp(msg),self.get_clock().now().nanoseconds,
                           v.linear.x,v.linear.y,v.angular.z,p.x,p.y,yaw)
        def ack(self,msg):
            try:
                value=self.json_value(msg)
                if value.get('id')==session+':'+str(trial.announcement_id):
                    trial.ack(trial.announcement_id,value.get('success'),time.monotonic())
            except (ValueError,TypeError):pass
        def stop(self,request,response):
            trial.stop(time.monotonic());self.publish((0.,0.,0.))
            response.success,response.message=True,'实验已停止';return response
        def publish(self,values):
            msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
            msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z=values;self.command_pub.publish(msg)
            if any(values):self.command_count+=1
        def tick(self):
            now=time.monotonic()
            if args.stop_file.exists():trial.stop(now)
            if writer.status()['error']:trial.fail('event_writer_failure',now)
            if trial.phase=='waiting' and now-self.started>10:trial.fail('inputs_not_ready_before_start',now)
            values=trial.tick(now,self.get_clock().now().nanoseconds);self.publish(values)
            if trial.phase=='yield' and not trial.announcement_ack and now-self.last_speech>=.5:
                self.speech.publish(String(data=json.dumps({'id':session+':'+str(trial.announcement_id),
                                         'kind':'crosswalk','text':'人行道前停车礼让行人'})))
                self.last_speech=now
            for event in trial.events:
                if not writer.submit(dict(event,session=session)):trial.fail('event_queue_full',now)
                if event['type'] in ('crosswalk_stop','crosswalk_stop_complete','stop','fault','start') and self.latest_image:
                    name=event['type']+'-'+str(self.latest_image_stamp)+'.jpg'
                    if name not in self.saved_snapshots:
                        self.saved_snapshots.add(name)
                        if not writer.submit(dict(type='snapshot',at=now,image_path=name,image_stamp_ns=self.latest_image_stamp,
                                event=event['type'],_frame_bytes=self.latest_image)):
                            trial.fail('snapshot_queue_failure',now)
            trial.events=[]
            if now-self.last_status>=.2:
                status=dict(trial.status(now),view_quality=self.view_quality,session=session,start_anchor=args.start_landmark,
                            perception_buffer=self.perception_worker.stats());self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False)))
                writer.submit(dict(type='control_sample',at=now,**status));self.last_status=now
            if trial.phase in ('fault','done'):
                if self.ended is None:self.ended=now
                if trial.stationary() and trial.odom_at is not None and now-trial.odom_at<=.3:
                    if self.stopped_since is None:self.stopped_since=now
                else:self.stopped_since=None
                if ((self.stopped_since is not None and now-self.stopped_since>=.25) or now-self.ended>1.5):
                    self.done=True
    node=Test()
    try:
        deadline=time.monotonic()+args.seconds+15
        while not node.done and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        node.publish((0.,0.,0.))
        result=trial.status(time.monotonic())
        result.update(session=session,physical_motor_test=True,nonzero_publications=node.command_count,
                      stop_feedback_confirmed=node.stopped_since is not None,sonar_mode='diagnostic_only',
                      scope='operator-supervised visual trial; not official race timing')
        with (args.output_dir/'result.json').open('x') as out:json.dump(result,out,ensure_ascii=False,indent=2);out.write('\n');out.flush();os.fsync(out.fileno())
        print(json.dumps(result,ensure_ascii=False),flush=True)
    finally:
        for _ in range(3):node.publish((0.,0.,0.));time.sleep(.03)
        node.perception_worker.close();writer.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
