#!/usr/bin/env python3
"""Preview steering-model output from ROS 2 images; never publishes velocity."""
import argparse
import io
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PIL import Image
import torch
from automation.realtime import LatestWorker
from automation.steering_candidate import fresh_frame
from training.train_steering import AutoPilot, preprocess_rgb


def load_model(path):
    path = Path(path).expanduser()
    if not path.is_absolute() or not path.is_file():
        raise ValueError('请指定已存在的绝对模型路径')
    model = AutoPilot()
    model.load_state_dict(torch.load(str(path), map_location='cpu', weights_only=True), strict=True)
    model.eval()
    return model


def predict_jpeg(model, data):
    if not data or len(data) > 10_000_000 or not data.startswith(b'\xff\xd8'):
        raise ValueError('无效的 JPEG 图像')
    with Image.open(io.BytesIO(data)) as picture:
        if picture.width <= 0 or picture.height <= 0 or picture.width * picture.height > 16_000_000:
            raise ValueError('图像尺寸无效')
        image = preprocess_rgb(picture).unsqueeze(0)
    start = time.monotonic()
    with torch.no_grad():
        steering = float(model(image).item())
    if not math.isfinite(steering):
        raise ValueError('模型输出无效')
    return steering, (time.monotonic() - start) * 1000


def main():
    p = argparse.ArgumentParser(description='循迹模型 ROS 2 只读预览，不输出底盘指令')
    p.add_argument('--model', required=True)
    p.add_argument('--camera-topic', default='/camera/rgb/image_raw/compressed')
    p.add_argument('--output-topic', default='/visual_race/steering_preview')
    p.add_argument('--max-fps', type=float, default=5.0)
    p.add_argument('--max-source-age', type=float, default=.8)
    p.add_argument('--threads', type=int, default=2)
    args = p.parse_args()
    if not math.isfinite(args.max_fps) or not 0 < args.max_fps <= 10 or not math.isfinite(args.max_source_age) or args.max_source_age <= 0 or not 1 <= args.threads <= 8:
        p.error('推理频率、时效或线程参数无效')
    torch.set_num_threads(args.threads)
    model = load_model(args.model)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
    except ImportError as error:
        raise SystemExit('需要 ROS 2 环境：' + str(error))

    class Preview(Node):
        def __init__(self):
            super().__init__('visual_race_steering_preview')
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.pub = self.create_publisher(String, args.output_topic, 1)
            sensor_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(CompressedImage, args.camera_topic, self.frame, sensor_qos)
            self.create_timer(.02, self.publish_result, clock=self.steady_clock)
            self.create_timer(.25, self.check_timeout, clock=self.steady_clock)
            self.last_good = None
            self.last_stamp_ns = 0
            self.reject_through_ns = 0
            self.worker = LatestWorker(self.process_frame, args.max_fps)
            self.worker.start()

        def send(self, value):
            self.pub.publish(String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))

        def frame(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if (stamp <= self.last_stamp_ns or
                    not fresh_frame(stamp, self.get_clock().now().nanoseconds, args.max_source_age)):
                return
            self.last_stamp_ns = stamp
            if not 0 < len(msg.data) <= 10_000_000:
                self.reject_through_ns = stamp
                self.send({'source': 'steering_cnn', 'valid': False})
                return
            self.worker.submit((stamp, bytes(msg.data)))

        @staticmethod
        def process_frame(frame):
            stamp, data = frame
            started = time.monotonic()
            steering, inference_ms = predict_jpeg(model, data)
            return {'image_stamp_ns': stamp, 'steering': steering, 'inference_ms': inference_ms,
                    'processing_ms': (time.monotonic() - started) * 1000}

        def publish_result(self):
            result = self.worker.take()
            if result is None:
                return
            if not result['ok']:
                self.send({'source': 'steering_cnn', 'valid': False})
                self.get_logger().warning('循迹预览失败: ' + result['error'], throttle_duration_sec=3.0)
                return
            value = result['value']
            stamp = value['image_stamp_ns']
            current_ns = self.get_clock().now().nanoseconds
            if stamp <= self.reject_through_ns or not fresh_frame(stamp, current_ns, args.max_source_age):
                self.send({'source': 'steering_cnn', 'valid': False})
                return
            self.last_good = time.monotonic()
            self.send({'source': 'steering_cnn', 'valid': True, 'steering': value['steering'],
                       'image_stamp_ns': stamp, 'image_age_ms': round((current_ns - stamp) / 1e6, 2),
                       'processing_ms': round(value['processing_ms'], 2),
                       'inference_ms': round(value['inference_ms'], 2),
                       'frames_replaced': self.worker.stats()['input_replaced']})

        def check_timeout(self):
            if self.last_good is None or time.monotonic() - self.last_good > .5:
                self.send({'source': 'steering_cnn', 'valid': False})

    rclpy.init()
    node = Preview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.worker.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
