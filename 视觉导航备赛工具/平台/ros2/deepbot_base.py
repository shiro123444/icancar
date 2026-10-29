#!/usr/bin/env python3
"""ROS 2 DEEPBOT serial adapter. Defaults to sensor-only mode."""
import argparse
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.base_protocol import FrameDecoder, measured_velocity, velocity_frame


def main():
    p=argparse.ArgumentParser(description='ROS 2 DEEPBOT 底盘接口，默认仅读取反馈')
    p.add_argument('--device',required=True)
    p.add_argument('--odom-topic',default='/raw_odom')
    p.add_argument('--cmd-topic',default='/cmd_vel')
    p.add_argument('--enable-drive',action='store_true')
    p.add_argument('--max-linear',type=float,default=.15)
    p.add_argument('--max-angular',type=float,default=.5)
    p.add_argument('--cmd-timeout',type=float,default=.3)
    p.add_argument('--publish-hz',type=float,default=20.,help='里程计发布频率上限')
    args=p.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (args.max_linear,args.max_angular,args.cmd_timeout,args.publish_hz)) or args.publish_hz>100:
        p.error('速度或超时无效')
    import serial
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    port=serial.Serial(args.device,115200,timeout=0,write_timeout=.2,exclusive=True)
    decoder=FrameDecoder()
    class Base(Node):
        def __init__(self):
            super().__init__('visual_race_deepbot_base')
            self.pub=self.create_publisher(Odometry,args.odom_topic,qos_profile_sensor_data)
            self.x=self.y=self.yaw=0.0
            self.last_measurement=None
            self.last_publish=None
            self.command=(0.0,0.0,0.0)
            self.command_at=None
            if args.enable_drive:
                self.create_subscription(Twist,args.cmd_topic,self.command_cb,10)
                self.create_timer(.1,self.send_command)
            self.create_timer(.01,self.receive)
        def command_cb(self,msg):
            values=(msg.linear.x,msg.linear.y,msg.angular.z)
            if not all(math.isfinite(v) for v in values):
                self.command,self.command_at=(0.,0.,0.),None
                return
            vx,vy,w=values
            speed=math.hypot(vx,vy)
            if speed>args.max_linear:vx,vy=vx*args.max_linear/speed,vy*args.max_linear/speed
            self.command=(vx,vy,max(-args.max_angular,min(args.max_angular,w)))
            self.command_at=time.monotonic()
        def send_command(self):
            now=time.monotonic()
            command=self.command if self.command_at is not None and 0<=now-self.command_at<=args.cmd_timeout else (0.,0.,0.)
            try:port.write(velocity_frame(*command))
            except (OSError,serial.SerialException) as e:self.get_logger().error('底盘串口发送失败: '+str(e))
        def receive(self):
            try:raw=port.read(min(port.in_waiting,8192))
            except (OSError,serial.SerialException) as e:
                self.get_logger().error('底盘串口读取失败: '+str(e))
                return
            for frame in decoder.feed(raw):
                try:vx,vy,w=measured_velocity(frame)
                except ValueError:continue
                now=time.monotonic()
                dt=0 if self.last_measurement is None else now-self.last_measurement
                self.last_measurement=now
                if 0<dt<.25:
                    self.x+=(vx*math.cos(self.yaw)-vy*math.sin(self.yaw))*dt
                    self.y+=(vx*math.sin(self.yaw)+vy*math.cos(self.yaw))*dt
                    self.yaw=math.atan2(math.sin(self.yaw+w*dt),math.cos(self.yaw+w*dt))
                if self.last_publish is not None and now-self.last_publish < 1.0/args.publish_hz:
                    continue
                self.last_publish=now
                msg=Odometry()
                msg.header.stamp=self.get_clock().now().to_msg()
                msg.header.frame_id='odom'
                msg.child_frame_id='base_link'
                msg.pose.pose.position.x,msg.pose.pose.position.y=self.x,self.y
                msg.pose.pose.orientation.z=math.sin(self.yaw/2)
                msg.pose.pose.orientation.w=math.cos(self.yaw/2)
                msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z=vx,vy,w
                self.pub.publish(msg)
    rclpy.init()
    node=Base()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if args.enable_drive:
            try:port.write(velocity_frame(0.,0.,0.))
            except (OSError,serial.SerialException):pass
        port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
