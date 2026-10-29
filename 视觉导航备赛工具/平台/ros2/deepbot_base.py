#!/usr/bin/env python3
"""ROS 2 DEEPBOT serial adapter. Defaults to sensor-only mode."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.base_protocol import FrameDecoder, measured_sensors, measured_velocity, velocity_frame
from automation.command import VelocityBuffer


def main():
    p=argparse.ArgumentParser(description='ROS 2 DEEPBOT 底盘接口，默认仅读取反馈')
    p.add_argument('--device',required=True)
    p.add_argument('--odom-topic',default='/raw_odom')
    p.add_argument('--sensor-topic',default='',help='可选的厂家原始电池和测距数值观察话题，不作为安全许可')
    p.add_argument('--cmd-topic',default='/cmd_vel')
    p.add_argument('--cmd-type',choices=('twist-stamped','twist'),default='twist-stamped',
                   help='默认使用带仲裁时间的指令；twist 仅用于兼容旧驱动')
    p.add_argument('--enable-drive',action='store_true')
    p.add_argument('--max-linear',type=float,default=.15)
    p.add_argument('--max-angular',type=float,default=.5)
    p.add_argument('--cmd-timeout',type=float,default=.3)
    p.add_argument('--cmd-max-age',type=float,default=.3,help='底盘接收和发送时允许的指令最大年龄（秒）')
    p.add_argument('--publish-hz',type=float,default=20.,help='里程计发布频率上限')
    args=p.parse_args()
    if args.sensor_topic and not args.sensor_topic.startswith('/visual_race/'):
        p.error('原始传感器观察话题必须位于 /visual_race/ 下')
    if (not all(math.isfinite(v) and v>0 for v in (args.max_linear,args.max_angular,args.cmd_timeout,args.cmd_max_age,args.publish_hz))
            or args.publish_hz>100 or args.max_linear>32 or args.max_angular>32):
        p.error('速度或超时无效')
    import serial
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist,TwistStamped
    from std_msgs.msg import String
    port=serial.Serial(args.device,115200,timeout=0,write_timeout=.2,exclusive=True)
    decoder=FrameDecoder()
    class Base(Node):
        def __init__(self):
            super().__init__('visual_race_deepbot_base')
            self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
            self.pub=self.create_publisher(Odometry,args.odom_topic,qos_profile_sensor_data)
            self.sensor_pub=self.create_publisher(String,args.sensor_topic,1) if args.sensor_topic else None
            self.x=self.y=self.yaw=0.0
            self.last_measurement=None
            self.last_publish=None
            self.last_sensor_publish=None
            self.commands=VelocityBuffer(args.max_linear,args.max_angular,args.cmd_timeout,
                                         args.cmd_max_age,stamped=args.cmd_type=='twist-stamped')
            if args.enable_drive:
                command_type=TwistStamped if args.cmd_type=='twist-stamped' else Twist
                self.create_subscription(command_type,args.cmd_topic,self.command_cb,1)
                self.create_timer(.1,self.send_command,clock=self.steady_clock)
                if args.cmd_type=='twist':self.get_logger().warning('兼容模式无法检查指令的来源时间')
            self.create_timer(.01,self.receive,clock=self.steady_clock)
        def command_cb(self,msg):
            if args.cmd_type=='twist-stamped':
                stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                twist=msg.twist
            else:
                stamp,twist=None,msg
            now,ros_now=time.monotonic(),self.get_clock().now().nanoseconds
            self.commands.update(twist.linear.x,twist.linear.y,twist.angular.z,now,stamp,ros_now)
            if self.commands.output(now,ros_now)==(0.,0.,0.):
                self.send_command()
        def send_command(self):
            command=self.commands.output(time.monotonic(),self.get_clock().now().nanoseconds)
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
                if self.sensor_pub is not None and (self.last_sensor_publish is None or now-self.last_sensor_publish>=.2):
                    values=measured_sensors(frame)
                    values.update({'source':'deepbot_base_raw','stamp_ns':self.get_clock().now().nanoseconds})
                    self.sensor_pub.publish(String(data=json.dumps(values,allow_nan=False)))
                    self.last_sensor_publish=now
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
