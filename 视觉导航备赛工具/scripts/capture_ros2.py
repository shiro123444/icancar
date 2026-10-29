#!/usr/bin/env python3
"""Capture frames from an explicitly selected ROS 2 sensor_msgs/Image topic.
This tool never publishes velocity commands.
"""
import argparse
from pathlib import Path
import time

def main():
    p = argparse.ArgumentParser(description='从指定 ROS 2 图像话题采集图片，不控制车辆。')
    p.add_argument('--topic', required=True, help='已验证的 sensor_msgs/msg/Image 话题')
    p.add_argument('--out', required=True, help='图片输出目录')
    p.add_argument('--interval', type=float, default=1.0, help='采集间隔秒，默认 1')
    p.add_argument('--count', type=int, default=100, help='最大图片数量，默认 100')
    args = p.parse_args()
    if args.interval <= 0 or args.count <= 0:
        p.error('interval 和 count 必须大于零')
    try:
        import cv2
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError as exc:
        p.exit(2, f'缺少 ROS 2、cv_bridge 或 OpenCV：{exc}\n请使用已配置 ROS 2 的 Python 环境。\n')
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    class Capture(Node):
        def __init__(self):
            super().__init__('competition_frame_capture')
            self.bridge = CvBridge()
            self.last = 0.0
            self.count = 0
            self.sub = self.create_subscription(Image, args.topic, self.callback, qos_profile_sensor_data)
            self.get_logger().info(f'监听 {args.topic}，输出到 {out.resolve()}')
        def callback(self, msg):
            now = time.monotonic()
            if now - self.last < args.interval:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                path = out / f'frame_{time.time_ns()}.jpg'
                if not cv2.imwrite(str(path), frame):
                    raise RuntimeError('图片写入失败')
                self.count += 1
                self.last = now
                self.get_logger().info(f'已保存 {self.count}/{args.count}: {path}')
            except Exception as exc:
                self.get_logger().error(f'保存失败: {exc}')
    rclpy.init()
    node = Capture()
    try:
        while rclpy.ok() and node.count < args.count:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
