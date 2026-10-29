#!/usr/bin/env python3
"""Isolated ROS 2 barrier and range-interlock replay; no real devices."""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '78' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('仅允许在本机隔离 ROS 2 域 78 中运行')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    root = Path(__file__).resolve().parents[1]
    def scene(board):
        image=np.full((240,320,3),130,dtype=np.uint8)
        image[:,:60]=(0,60,220);image[:,260:] = (0,60,220)
        image[:,60:68]=(250,220,0);image[:,252:260]=(250,220,0)
        if board=='avoid':image[70:170,75:150]=(0,55,210)
        if board=='near':image[70:230,75:150]=(0,55,210)
        ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
        if not ok:raise ValueError('synthetic JPEG encoding failed')
        return encoded.tobytes()
    images={name:scene(name) for name in ('clear','avoid','near')}
    with tempfile.TemporaryDirectory(prefix='visual-race-barrier-smoke-') as tmp:
        folder=Path(tmp);configs={}
        for name in ('barrier','avoidance','range-safety'):
            value=json.loads((root/'data'/(name+'.example.json')).read_text(encoding='utf-8'))
            value['calibrated']=True  # Synthetic test only.
            configs[name]=folder/(name+'.json');configs[name].write_text(json.dumps(value),encoding='utf-8')
        topics={'image':'/visual_race/trial/barrier_image','obs':'/visual_race/trial/barrier_obs',
                'lane':'/visual_race/trial/lane','candidate':'/visual_race/trial/avoid_candidate',
                'range':'/visual_race/trial/range','safety':'/visual_race/trial/range_safety',
                'odom':'/visual_race/trial/barrier_odom','estop':'/visual_race/trial/barrier_estop',
                'status':'/visual_race/trial/barrier_status','output':'/visual_race/trial/barrier_output',
                'reset':'/visual_race/trial/barrier_reset','diagnostics':'/visual_race/trial/barrier_diagnostics',
                'avoidance_diag':'/visual_race/trial/avoidance_diagnostics'}
        commands=[
            [sys.executable,str(root/'ros2/barrier_detector.py'),'--config',str(configs['barrier']),
             '--camera-topic',topics['image'],'--output-topic',topics['obs']],
            [sys.executable,str(root/'ros2/barrier_avoidance.py'),'--config',str(configs['avoidance']),'--activate',
             '--lane-topic',topics['lane'],'--observation-topic',topics['obs'],'--output-topic',topics['candidate'],
             '--diagnostics-topic',topics['avoidance_diag']],
            [sys.executable,str(root/'ros2/range_safety.py'),'--config',str(configs['range-safety']),
             '--range-topic',topics['range'],'--safety-topic',topics['safety']],
            [sys.executable,str(root/'ros2/command_gate.py'),'--activate','--input-topic',topics['candidate'],
             '--output-topic',topics['output'],'--status-topic',topics['status'],'--safety-topic',topics['safety'],
             '--feedback-topic',topics['odom'],'--estop-topic',topics['estop'],'--reset-service',topics['reset'],
             '--diagnostics-topic',topics['diagnostics'],'--max-linear','.05','--max-angular','.4']]
        with tempfile.TemporaryFile(mode='w+t') as log:
            children=[subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT) for cmd in commands]
            rclpy.init()
            class Trial(Node):
                def __init__(self):
                    super().__init__('visual_race_barrier_replay')
                    self.camera=self.create_publisher(CompressedImage,topics['image'],qos_profile_sensor_data)
                    self.range=self.create_publisher(Range,topics['range'],qos_profile_sensor_data)
                    self.odom=self.create_publisher(Odometry,topics['odom'],qos_profile_sensor_data)
                    self.lane=self.create_publisher(TwistStamped,topics['lane'],1)
                    self.estop=self.create_publisher(Bool,topics['estop'],1)
                    self.status=self.create_publisher(String,topics['status'],1)
                    self.create_subscription(TwistStamped,topics['output'],self.output,1)
                    self.create_subscription(String,topics['avoidance_diag'],self.avoidance_diagnostics,1)
                    self.create_subscription(Bool,topics['safety'],self.safety_status,1)
                    self.reset=self.create_client(Trigger,topics['reset'])
                    self.future=None;self.reset_done=False
                    self.avoidance_reason,self.safety_clear=None,None
                    self.stage,self.changed='warmup',time.monotonic()
                    self.checks={k:False for k in ('forward','avoidance','barrier_stop','range_stop','range_recovery','range_loss_stop')}
                    self.create_timer(.1,self.tick)
                def change(self,stage):self.stage,self.changed=stage,time.monotonic()
                def avoidance_diagnostics(self,msg):
                    try:self.avoidance_reason=json.loads(msg.data).get('reason')
                    except (ValueError,TypeError,AttributeError):pass
                def safety_status(self,msg):self.safety_clear=msg.data
                def tick(self):
                    stamp=self.get_clock().now().to_msg()
                    image=CompressedImage();image.header.stamp=stamp;image.format='jpeg'
                    image.data=images['avoid' if self.stage=='avoidance' else 'near' if self.stage=='barrier_stop' else 'clear']
                    self.camera.publish(image)
                    odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.;self.odom.publish(odom)
                    lane=TwistStamped();lane.header.stamp=stamp;lane.twist.linear.x=.05;self.lane.publish(lane)
                    self.estop.publish(Bool(data=False))
                    self.status.publish(String(data=json.dumps({'phase':'running','action':'follow_lane','calibrated':True})))
                    if self.stage!='range_loss_stop':
                        measurement=Range();measurement.header.stamp=stamp;measurement.min_range=.05;measurement.max_range=2.
                        measurement.range=.2 if self.stage=='range_stop' else 1.;self.range.publish(measurement)
                    if self.future is not None and self.future.done():
                        self.reset_done=bool(self.future.result().success);self.future=None
                    if not self.reset_done and self.future is None and self.reset.service_is_ready():
                        self.future=self.reset.call_async(Trigger.Request())
                def output(self,msg):
                    x,y,w=msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z
                    if not all(math.isfinite(v) for v in (x,y,w)) or math.hypot(x,y)>.050001 or abs(w)>.400001:
                        raise ValueError('isolated output exceeded configured limits')
                    moving=abs(x)>.001
                    elapsed=time.monotonic()-self.changed
                    if self.stage=='warmup' and moving:
                        self.checks['forward']=True;self.change('avoidance')
                    elif self.stage=='avoidance' and moving and w<-.01 and self.avoidance_reason=='avoid_barrier':
                        self.checks['avoidance']=True;self.change('barrier_stop')
                    elif self.stage=='barrier_stop' and not moving and elapsed>.4 and self.avoidance_reason=='barrier_too_close':
                        self.checks['barrier_stop']=True;self.change('range_stop')
                    elif self.stage=='range_stop' and not moving and elapsed>.4 and self.safety_clear is False:
                        self.checks['range_stop']=True;self.change('range_recovery')
                    elif self.stage=='range_recovery' and moving and self.safety_clear is True:
                        self.checks['range_recovery']=True;self.change('range_loss_stop')
                    elif self.stage=='range_loss_stop' and not moving and elapsed>.7 and self.safety_clear is False:
                        self.checks['range_loss_stop']=True
            node=None
            try:
                node=Trial()
                deadline=time.monotonic()+40
                while time.monotonic()<deadline and not node.checks['range_loss_stop'] and all(c.poll() is None for c in children):
                    rclpy.spin_once(node,timeout_sec=.2)
                result={'checks':node.checks,'all_passed':all(node.checks.values()),
                        'simulated_camera_and_range':True,'physical_commands':False}
                if not result['all_passed']:
                    result['stage']=node.stage;log.seek(0);result['logs']=log.read()[-3000:]
                print(json.dumps(result,ensure_ascii=False),flush=True)
                if not result['all_passed']:raise SystemExit(1)
            finally:
                if node is not None:node.destroy_node()
                rclpy.shutdown()
                for child in children:child.terminate()
                for child in children:
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:child.kill();child.wait()


if __name__=='__main__':main()
