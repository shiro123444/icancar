#!/usr/bin/env python3
"""Exercise steering preview using a saved JPEG in isolated ROS 2 domain 78."""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='隔离 ROS 2 域中的循迹模型只读回放')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '78' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('仅允许在本机隔离 ROS 2 域 78 中运行')
    model, image = args.model.resolve(), args.image.read_bytes()
    if not model.is_file() or not image.startswith(b'\xff\xd8'):
        raise SystemExit('模型或 JPEG 图片不存在')
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    preview = Path(__file__).resolve().parents[1] / 'ros2' / 'steering_preview.py'
    camera_topic, output_topic = '/trial_steering/camera', '/trial_steering/output'
    with tempfile.TemporaryFile(mode='w+t') as log:
        child = subprocess.Popen([sys.executable, str(preview), '--model', str(model),
                                  '--camera-topic', camera_topic, '--output-topic', output_topic],
                                 stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        class Trial(Node):
            def __init__(self):
                super().__init__('isolated_steering_replay')
                self.pub = self.create_publisher(CompressedImage, camera_topic, qos_profile_sensor_data)
                self.create_subscription(String, output_topic, self.result, 10)
                self.create_timer(.2, self.frame)
                self.enabled, self.good, self.stale = True, None, False
            def frame(self):
                if not self.enabled:
                    return
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format, msg.data = 'jpeg', image
                self.pub.publish(msg)
            def result(self, msg):
                try:
                    value = json.loads(msg.data)
                    if value.get('valid') is True:
                        self.good, self.enabled = value, False
                    elif self.good is not None and value.get('valid') is False:
                        self.stale = True
                except (ValueError, AttributeError):
                    pass
        node = Trial()
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline and not node.stale and child.poll() is None:
                rclpy.spin_once(node, timeout_sec=.2)
            if node.good is None or not node.stale:
                log.seek(0)
                raise SystemExit('只读回放失败: ' + log.read()[-2000:])
            print(json.dumps({'preview': node.good, 'stale_image_detected': node.stale,
                              'domain': 78, 'motor_commands': False}, ensure_ascii=False))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
