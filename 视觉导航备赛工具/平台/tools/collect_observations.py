#!/usr/bin/env python3
"""Read-only ROS 2 observation capture. No publishers, services or serial I/O."""
import argparse
import json
import math
import os
import time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description='只读采集相机、里程计和模型观察，不发布控制指令')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600)
    p.add_argument('--interval',type=float,default=2)
    args=p.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds<=900:p.error('时长须在 0–900 秒内')
    if not math.isfinite(args.interval) or not .5<=args.interval<=10:p.error('间隔须为 0.5–10 秒')
    if args.output.exists():p.error('采集目录已存在')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    args.output.mkdir(parents=True);(args.output/'frames').mkdir()
    rclpy.init()
    class Capture(Node):
        def __init__(self):
            super().__init__('visual_race_readonly_observation_capture')
            self.image=self.odom=self.steering=self.model=None
            self.frames=self.stationary_frames=0;self.last_saved=0
            self.create_subscription(CompressedImage,'/camera/rgb/image_raw/compressed',self.camera,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/raw_odom',self.odometry,qos_profile_sensor_data)
            self.create_subscription(String,'/visual_race/steering_preview',lambda m:self.store('steering',m),1)
            self.create_subscription(String,'/visual_race/model_detections',lambda m:self.store('model',m),1)
        def camera(self,msg):
            if 0<len(msg.data)<=10000000:
                self.image=(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,bytes(msg.data))
        def odometry(self,msg):
            v=msg.twist.twist;q=msg.pose.pose.orientation;p=msg.pose.pose.position
            values=[v.linear.x,v.linear.y,v.angular.z,p.x,p.y,q.x,q.y,q.z,q.w]
            if all(math.isfinite(n) for n in values):
                self.odom=dict(stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                               velocity=values[:3],position=values[3:5],quaternion=values[5:])
        def store(self,name,msg):
            try:
                if len(msg.data)>150000:return
                value=json.loads(msg.data,parse_constant=lambda _:(_ for _ in ()).throw(ValueError('invalid JSON')))
                setattr(self,name,value)
            except (TypeError,ValueError):pass
        def save(self):
            if self.image is None or self.image[0]<=self.last_saved:return
            stamp,data=self.image
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if not -.2<=age<=1:return
            self.last_saved=stamp;name='frames/'+str(stamp)+'.jpg'
            (args.output/name).write_bytes(data)
            v=None if self.odom is None else self.odom['velocity']
            stationary=v is not None and math.hypot(*v[:2])<=.012 and abs(v[2])<=.06
            record=dict(image_path=name,image_stamp_ns=stamp,capture_mode='stationary_observation',
                        training_label_valid=False,stationary=stationary,odometry=self.odom,
                        steering_prediction=self.steering,perception_model=self.model)
            with (args.output/'observations.jsonl').open('a') as out:out.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
            self.frames+=1;self.stationary_frames+=int(stationary)
    node=Capture();started=time.monotonic();saved=started
    try:
        while time.monotonic()-started<args.seconds:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-saved>=args.interval:node.save();saved=time.monotonic()
    except KeyboardInterrupt:pass
    finally:
        result=dict(read_only=True,motion_commands_sent=0,frames=node.frames,stationary_frames=node.stationary_frames,
                    training_label_valid=False,elapsed_seconds=round(time.monotonic()-started,2))
        with (args.output/'summary.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n');out.flush();os.fsync(out.fileno())
        print(json.dumps(result),flush=True);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
