#!/usr/bin/env python3
"""Replay recorded images through a ROS 2 controller in isolated domain 78.

This publishes only test images, zero odometry and simulated speech receipts.
It never publishes velocity or connects to the vehicle control domain.
"""
import argparse
import io
import json
import os
import time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description='隔离环境中的两圈与人行道事件联调')
    parser.add_argument('--line',required=True)
    parser.add_argument('--crosswalk',required=True)
    parser.add_argument('--timeout',type=float,default=50)
    parser.add_argument('--check-gate',action='store_true',help='在隔离话题验证速度仲裁')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='78':
        raise SystemExit('仅允许在隔离测试域 ROS_DOMAIN_ID=78 运行')
    from PIL import Image as PILImage
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Bool
    from std_srvs.srv import Trigger
    buffer=io.BytesIO()
    PILImage.new('RGB',(640,480),(0,0,0)).save(buffer,format='JPEG')
    scenes={'clear':buffer.getvalue(),'line':Path(args.line).read_bytes(),'crosswalk':Path(args.crosswalk).read_bytes()}
    rclpy.init()
    class Replay(Node):
        def __init__(self):
            super().__init__('visual_race_isolated_replay')
            self.images=self.create_publisher(CompressedImage,'/trial/camera',qos_profile_sensor_data)
            self.odom=self.create_publisher(Odometry,'/trial/odom',qos_profile_sensor_data)
            self.acks=self.create_publisher(String,'/trial/speech_ack',10)
            self.candidate=self.create_publisher(Twist,'/trial/candidate',10) if args.check_gate else None
            self.safety=self.create_publisher(Bool,'/trial/safety',10) if args.check_gate else None
            self.create_subscription(String,'/trial/status',self.on_status,10)
            self.create_subscription(String,'/trial/speech',self.on_speech,10)
            if args.check_gate:self.create_subscription(Twist,'/trial/output',self.on_command,10)
            self.arm=self.create_client(Trigger,'/visual_race/arm')
            self.create_timer(.1,self.tick)
            self.state={}
            self.scene='clear'
            self.step='warmup'
            self.future=None
            self.acks_sent=set()
            self.moving_candidate_seen=False
            self.yield_stops=set()
            self.final_stop=False
            self.finish=None
        def on_command(self,msg):
            speed=abs(msg.linear.x)+abs(msg.linear.y)+abs(msg.angular.z)
            if speed>1e-6 and self.state.get('phase') in ('armed','running'):
                self.moving_candidate_seen=True
            if speed<=1e-6 and self.state.get('phase')=='yield':
                self.yield_stops.add(self.state.get('announcement_id'))
            if speed<=1e-6 and self.state.get('phase')=='finished':
                self.final_stop=True
        def on_status(self,msg):
            try:self.state=json.loads(msg.data)
            except ValueError:pass
        def on_speech(self,msg):
            try:
                value=json.loads(msg.data)
                if value.get('kind')=='crosswalk' and value.get('id'):
                    identifier=value['id']
                    self.acks.publish(String(data=json.dumps({'id':identifier,'success':True})))
                    self.acks_sent.add(identifier)
            except (ValueError,TypeError):pass
        def change(self,step,scene):
            self.step,self.scene=step,scene
            print('test stage:',step,flush=True)
        def tick(self):
            stamp=self.get_clock().now().to_msg()
            image=CompressedImage()
            image.header.stamp=stamp
            image.format='jpeg'
            image.data=scenes[self.scene]
            odom=Odometry()
            odom.header.stamp=stamp
            odom.pose.pose.orientation.w=1.0
            self.images.publish(image)
            self.odom.publish(odom)
            if args.check_gate:
                candidate=Twist()
                candidate.linear.x=.1
                self.candidate.publish(candidate)
                self.safety.publish(Bool(data=True))
            s=self.state
            if s.get('phase') in ('yield','finished') and s.get('action')!='stop':
                self.finish={'error':'controller did not request stop during '+str(s['phase'])}
                return
            clear=s.get('line') is False and s.get('crosswalk') is False
            if self.step=='warmup' and clear and self.arm.service_is_ready():
                self.future=self.arm.call_async(Trigger.Request())
                self.change('arming','clear')
            elif self.step=='arming' and self.future.done():
                result=self.future.result()
                if not result.success:
                    self.finish={'error':'arm rejected: '+result.message}
                else:self.change('start','line')
            elif self.step=='start' and s.get('phase')=='running':self.change('before_crosswalk_1','clear')
            elif self.step=='before_crosswalk_1' and clear:self.change('crosswalk_1','crosswalk')
            elif self.step=='crosswalk_1' and s.get('phase')=='yield':self.change('waiting_1','crosswalk')
            elif self.step=='waiting_1' and s.get('phase')=='running' and s.get('crosswalk_stops',0)>=1:self.change('before_lap_1','clear')
            elif self.step=='before_lap_1' and clear:self.change('lap_1','line')
            elif self.step=='lap_1' and s.get('laps')==1:self.change('before_crosswalk_2','clear')
            elif self.step=='before_crosswalk_2' and clear:self.change('crosswalk_2','crosswalk')
            elif self.step=='crosswalk_2' and s.get('phase')=='yield':self.change('waiting_2','crosswalk')
            elif self.step=='waiting_2' and s.get('phase')=='running' and s.get('crosswalk_stops',0)>=2:self.change('before_lap_2','clear')
            elif self.step=='before_lap_2' and clear:self.change('lap_2','line')
            elif self.step=='lap_2' and s.get('phase')=='finished':self.change('finished','line')
            elif self.step=='finished' and (not args.check_gate or self.final_stop):
                self.finish={'phase':s['phase'],'laps':s['laps'],'crosswalk_stops':s['crosswalk_stops'],
                             'elapsed_seconds':s['elapsed_seconds'],'simulated_speech_acknowledgments':len(self.acks_sent)}
                if args.check_gate:
                    self.finish['gate']={'allowed_candidate':self.moving_candidate_seen,
                                         'yield_stops':len(self.yield_stops),'finish_stop':self.final_stop}
                    if not self.moving_candidate_seen or len(self.yield_stops)<2:
                        self.finish={'error':'gate did not verify motion and both crosswalk stops'}
            elif s.get('phase')=='fault':self.finish={'error':'controller fault: '+str(s.get('reason'))}
    node=Replay()
    deadline=time.monotonic()+args.timeout
    while rclpy.ok() and node.finish is None and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.2)
    result=node.finish or {'error':'test timed out at '+node.step}
    print(json.dumps(result,ensure_ascii=False),flush=True)
    node.destroy_node()
    rclpy.shutdown()
    if 'error' in result or result.get('laps')!=2 or result.get('crosswalk_stops')!=2:
        raise SystemExit(1)


if __name__=='__main__':main()
