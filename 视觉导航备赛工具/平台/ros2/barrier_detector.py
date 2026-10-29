#!/usr/bin/env python3
"""ROS 2 barrier candidates from fresh camera frames; no motion publisher."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.barrier_detection import detect_barriers, validate_config
from automation.realtime import LatestWorker
from automation.steering_candidate import fresh_frame


def main():
    p = argparse.ArgumentParser(description='赛道内挡板候选识别，不直接控制底盘')
    p.add_argument('--config', required=True)
    p.add_argument('--camera-topic', default='/camera/rgb/image_raw/compressed')
    p.add_argument('--output-topic', default='/visual_race/barrier_observation')
    p.add_argument('--max-fps', type=float, default=5)
    p.add_argument('--max-source-age', type=float, default=.4)
    p.add_argument('--timeout', type=float, default=.5)
    args = p.parse_args()
    if (not math.isfinite(args.max_fps) or not 0 < args.max_fps <= 10
            or not math.isfinite(args.max_source_age) or args.max_source_age <= 0
            or not math.isfinite(args.timeout) or args.timeout <= 0):
        p.error('频率或时效参数无效')
    if not args.output_topic.startswith('/visual_race/'):
        p.error('挡板识别输出必须位于 /visual_race/ 下')
    with open(args.config, encoding='utf-8') as handle:
        config = validate_config(json.load(handle))
    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
    except ImportError as error:
        raise SystemExit('需要 ROS 2、OpenCV 与 NumPy：' + str(error))

    class Detector(Node):
        def __init__(self):
            super().__init__('visual_race_barrier_detector')
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.pub = self.create_publisher(String, args.output_topic, 1)
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CompressedImage, args.camera_topic, self.frame, qos)
            self.last_stamp, self.reject_through, self.last_good = 0, 0, None
            self.worker = LatestWorker(self.process, args.max_fps)
            self.worker.start()
            self.create_timer(.02, self.publish_result, clock=self.steady_clock)
            self.create_timer(.2, self.check_timeout, clock=self.steady_clock)

        def send(self, value):
            self.pub.publish(String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))

        def frame(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= self.last_stamp or not fresh_frame(stamp, self.get_clock().now().nanoseconds, args.max_source_age):
                return
            self.last_stamp = stamp
            if not 0 < len(msg.data) <= 10_000_000 or bytes(msg.data[:2]) != b'\xff\xd8':
                self.reject_through, self.last_good = stamp, None
                self.send({'source':'barrier_detector', 'valid':False})
                return
            self.worker.submit((stamp, bytes(msg.data)))

        @staticmethod
        def process(frame):
            stamp, data = frame
            bgr = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None or not bgr.size or bgr.shape[0] * bgr.shape[1] > 16_000_000:
                raise ValueError('图像读取失败')
            started = time.monotonic()
            result = detect_barriers(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), config)
            result.update({'source':'barrier_detector', 'stamp_ns':stamp,
                           'processing_ms':round((time.monotonic()-started)*1000, 2)})
            return result

        def publish_result(self):
            result = self.worker.take()
            if result is None:
                return
            if not result['ok']:
                self.last_good = None
                self.send({'source':'barrier_detector', 'valid':False})
                self.get_logger().warning('挡板识别失败: ' + result['error'], throttle_duration_sec=3.0)
                return
            value = result['value']
            if value['stamp_ns'] <= self.reject_through or not fresh_frame(value['stamp_ns'], self.get_clock().now().nanoseconds, args.max_source_age):
                self.last_good = None
                self.send({'source':'barrier_detector', 'valid':False})
                return
            self.last_good = time.monotonic()
            value['frames_replaced'] = self.worker.stats()['input_replaced']
            self.send(value)

        def check_timeout(self):
            if self.last_good is None or time.monotonic() - self.last_good > args.timeout:
                self.send({'source':'barrier_detector', 'valid':False})

    rclpy.init()
    node = Detector()
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
