#!/usr/bin/env python3
"""Collect synchronized camera frames and Twist commands. Never publishes commands."""
import argparse
import math
import time
from pathlib import Path
from dataset_session import DatasetSession

def parse_args():
    p=argparse.ArgumentParser(description='只读采集 ROS 2 JPEG 图像及速度指令，输出独立数据集')
    p.add_argument('--camera-topic',required=True,help='已确认图像话题')
    p.add_argument('--camera-type',required=True,choices=('raw','compressed'))
    p.add_argument('--command-topic',required=True,help='人工遥控时确认过的速度指令话题')
    p.add_argument('--command-type',choices=('twist','twist-stamped'),default='twist')
    p.add_argument('--output',required=True,help='必须不存在的新目录')
    p.add_argument('--hz',type=float,default=5.0,help='最大保存频率，默认 5 Hz')
    p.add_argument('--max-command-age-ms',type=float,default=250,help='控制命令最大接收间隔，默认 250ms')
    p.add_argument('--max-images',type=int,default=3000)
    a=p.parse_args()
    if not math.isfinite(a.hz) or not 0.1<=a.hz<=20:p.error('hz 必须在 0.1–20 内')
    if not math.isfinite(a.max_command_age_ms) or not 1<=a.max_command_age_ms<=2000:p.error('max-command-age-ms 必须在 1–2000 内')
    if a.max_images<1:p.error('max-images 必须大于 0')
    return a

def main():
    args=parse_args()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image,CompressedImage
        from geometry_msgs.msg import Twist,TwistStamped
        if args.camera_type=='raw':
            import cv2
            from cv_bridge import CvBridge
    except ImportError as exc:
        raise SystemExit(f'ROS 2 依赖缺失：{exc}。raw 图像还需要 cv_bridge 和 OpenCV。')
    metadata={'format':'visual-nav-twist-v1','camera_topic':args.camera_topic,'camera_type':args.camera_type,
              'command_topic':args.command_topic,'command_type':args.command_type,'hz':args.hz,
              'max_command_age_ms':args.max_command_age_ms,'max_images':args.max_images,
              'label_source':'最近接收的控制指令，不是车身实测速度；时间差基于本进程单调时钟',
              'vendor_compatibility':'未验证，模型训练前需检查实际输入格式'}
    try:
        session=DatasetSession(args.output,metadata,min_interval_s=1/args.hz,max_command_age_s=args.max_command_age_ms/1000)
    except (OSError,ValueError) as exc:raise SystemExit(f'无法创建新数据集：{exc}')
    class Recorder(Node):
        def __init__(self):
            super().__init__('vision_dataset_recorder')
            self.bridge=CvBridge() if args.camera_type=='raw' else None
            self.create_subscription(Image if args.camera_type=='raw' else CompressedImage,args.camera_topic,self.on_image,qos_profile_sensor_data)
            self.create_subscription(Twist if args.command_type=='twist' else TwistStamped,args.command_topic,self.on_command,qos_profile_sensor_data)
            self.get_logger().info('只读采集已启动：'+str(session.path))
        def on_command(self,msg):
            cmd=msg.twist if args.command_type=='twist-stamped' else msg
            try:session.set_command(cmd.linear.x,cmd.linear.y,cmd.angular.z,time.monotonic())
            except ValueError as exc:self.get_logger().warning(str(exc))
        def on_image(self,msg):
            try:
                if args.camera_type=='compressed':
                    image=bytes(msg.data)
                    if not image.startswith(b'\xff\xd8'):return
                else:
                    arr=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                    ok,encoded=cv2.imencode('.jpg',arr)
                    if not ok:return
                    image=encoded.tobytes()
                stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                if session.add_frame(image,stamp,time.monotonic()) and session.count%25==0:
                    self.get_logger().info(f'已保存 {session.count} 张；无新鲜指令跳过 {session.skipped_stale} 帧')
            except (ValueError,OSError) as exc:
                self.get_logger().error(f'采集出错，停止：{exc}');self.failed=True
    try:
        rclpy.init();node=Recorder();node.failed=False
        try:
            while rclpy.ok() and session.count < args.max_images and not node.failed:
                rclpy.spin_once(node,timeout_sec=0.5)
        finally:
            node.destroy_node();rclpy.shutdown()
    except KeyboardInterrupt:pass
    finally:
        session.close()
        print(f'采集结束：{session.count} 张，因缺少新鲜命令跳过 {session.skipped_stale} 帧；目录 {session.path}')
if __name__=='__main__':main()
