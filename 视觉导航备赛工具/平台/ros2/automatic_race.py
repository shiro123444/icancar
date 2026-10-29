#!/usr/bin/env python3
"""ROS 2 adapter for image detection and two-lap mission; no motor publisher."""
import argparse
import io
import json
import math
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.detector import detect, validate_config
from automation.race import Race
from automation.realtime import EventWriter
import numpy as np
from PIL import Image as PILImage


def source_stamp_fresh(header, now_ns, max_age):
    try:
        sec, ns = header.stamp.sec, header.stamp.nanosec
        if not isinstance(sec, int) or not isinstance(ns, int) or sec < 0 or not 0 <= ns < 1_000_000_000:
            return False
        stamp = sec * 1_000_000_000 + ns
        if not stamp or not isinstance(now_ns, int) or not math.isfinite(max_age) or max_age <= 0:
            return False
        age = (now_ns - stamp) / 1_000_000_000
        return -0.2 <= age <= max_age
    except (AttributeError, TypeError):
        return False


def decode(msg, kind):
    data = bytes(msg.data)
    if not data or len(data) > 10_000_000:
        raise ValueError('empty or oversized image')
    if kind == 'compressed':
        with PILImage.open(io.BytesIO(data)) as picture:
            if picture.width * picture.height > 16_000_000:
                raise ValueError('image dimensions too large')
            return np.asarray(picture.convert('RGB'))
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None or msg.height <= 0 or msg.width <= 0 or msg.width * msg.height > 16_000_000:
        raise ValueError('unsupported raw image encoding or size')
    if msg.step < msg.width * channels or len(data) < msg.step * msg.height:
        raise ValueError('invalid raw image stride')
    rows = np.frombuffer(data, np.uint8, count=msg.step * msg.height).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return np.repeat(image, 3, axis=2)
    return image[:, :, ::-1] if msg.encoding == 'bgr8' else image


def main():
    p = argparse.ArgumentParser(description='视觉赛道两圈任务与相机识别（不发布底盘命令）')
    p.add_argument('--camera-topic', required=True)
    p.add_argument('--camera-type', required=True, choices=('compressed', 'raw'))
    p.add_argument('--odom-topic', required=True)
    p.add_argument('--config', required=True)
    p.add_argument('--status-topic', default='/visual_race/status')
    p.add_argument('--speech-topic', default='/visual_race/speech')
    p.add_argument('--speech-ack-topic', default='/visual_race/speech_ack')
    p.add_argument('--timeout', type=float, default=0.6)
    p.add_argument('--max-source-age', type=float, default=0.8, help='允许的图像与里程计源时间戳最大延迟（秒）')
    p.add_argument('--events-dir', default='', help='可选：保存车端自动任务事件的目录')
    args = p.parse_args()
    if not math.isfinite(args.max_source_age) or args.max_source_age <= 0:
        p.error('max-source-age 必须是正有限数')
    with open(args.config, encoding='utf-8') as f:
        config = validate_config(json.load(f))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CompressedImage
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError as e:
        raise SystemExit('需要小车的 ROS 2 环境及相应消息包：' + str(e))

    class Controller(Node):
        def __init__(self):
            super().__init__('visual_race_controller')
            self.race = Race(timeout=args.timeout)
            self.session = uuid.uuid4().hex
            self.event_path = self.new_event_path()
            self.event_writer = EventWriter(self.write_event) if self.event_path else None
            self.log_failure_reported = False
            self.finish_ack = False
            self.last_speech = 0.0
            self.last_detection = None
            self.pub = self.create_publisher(String, args.status_topic, 10)
            self.speech = self.create_publisher(String, args.speech_topic, 10)
            self.create_subscription(CompressedImage if args.camera_type == 'compressed' else Image,
                                     args.camera_topic, self.camera, qos_profile_sensor_data)
            self.create_subscription(Odometry, args.odom_topic, self.odom, qos_profile_sensor_data)
            self.create_subscription(String, args.speech_ack_topic, self.ack, 10)
            self.create_service(Trigger, '/visual_race/arm', self.arm)
            self.create_service(Trigger, '/visual_race/stop', self.stop)
            self.create_service(Trigger, '/visual_race/reset', self.reset)
            self.create_timer(0.1, self.update)

        def new_event_path(self):
            if not args.events_dir:
                return None
            path = Path(args.events_dir).expanduser()
            path.mkdir(parents=True, exist_ok=True)
            return path / ('visual-race-' + self.session + '.jsonl')

        def flush_events(self):
            for event in self.race.pop_events():
                record = dict(event, session=self.session, wall_time=datetime.now(timezone.utc).isoformat())
                line = json.dumps(record, ensure_ascii=False, allow_nan=False)
                self.get_logger().info(line)
                if self.event_writer and not self.event_writer.submit((self.event_path, line)):
                    self.fail_event_writer()

        @staticmethod
        def write_event(item):
            path, line = item
            with path.open('a', encoding='utf-8') as out:
                out.write(line + '\n')

        def fail_event_writer(self):
            if self.log_failure_reported:
                return
            self.log_failure_reported = True
            self.get_logger().error('任务事件写入异常，任务进入停止状态')
            self.race.phase, self.race.reason = 'fault', 'event_log_failure'
            self.race.events.append({'type': 'fault', 'reason': 'event_log_failure', 'at': time.monotonic()})

        def send(self, publisher, obj):
            msg = String()
            msg.data = json.dumps(obj, ensure_ascii=False, allow_nan=False)
            publisher.publish(msg)

        def camera(self, msg):
            try:
                if not source_stamp_fresh(msg.header, self.get_clock().now().nanoseconds, args.max_source_age):
                    raise ValueError('图像源时间戳过期或无效')
                detected = detect(decode(msg, args.camera_type), config)
                self.last_detection = detected
                self.race.frame(time.monotonic(), detected['start_line']['detected'], detected['crosswalk']['detected'])
            except Exception as e:
                self.get_logger().warning('图像读取或检测失败: ' + str(e), throttle_duration_sec=3.0)

        def odom(self, msg):
            try:
                if not source_stamp_fresh(msg.header, self.get_clock().now().nanoseconds, args.max_source_age):
                    raise ValueError('里程计源时间戳过期或无效')
                v = msg.twist.twist
                self.race.odom(time.monotonic(), v.linear.x, v.linear.y, v.angular.z)
            except (ValueError, AttributeError) as e:
                self.get_logger().warning('里程计反馈无效: ' + str(e), throttle_duration_sec=3.0)

        def ack(self, msg):
            try:
                data = json.loads(msg.data)
                if data.get('id') == self.session + ':finish' and data.get('success') is True:
                    self.finish_ack = True
                expected = self.session + ':' + str(self.race.announcement_id)
                if data.get('id') == expected:
                    self.race.acknowledge(time.monotonic(), self.race.announcement_id, data.get('success'))
            except (ValueError, AttributeError):
                pass

        def arm(self, request, response):
            try:
                if self.event_writer and self.event_writer.status()['error']:
                    raise ValueError('事件记录不可用，请检查后重新启动')
                if not config['calibrated']:
                    raise ValueError('先用现场图像标定配置')
                self.race.arm(time.monotonic())
                response.success, response.message = True, '任务已准备'
            except ValueError as e:
                response.success, response.message = False, str(e)
            return response

        def stop(self, request, response):
            self.race.phase, self.race.reason = 'fault', 'manual_stop'
            self.race.events.append({'type': 'fault', 'reason': 'manual_stop', 'at': time.monotonic()})
            response.success, response.message = True, '任务已停止'
            return response

        def reset(self, request, response):
            if self.event_writer and self.event_writer.status()['error']:
                response.success, response.message = False, '事件记录不可用，请检查后重新启动'
                return response
            if self.race.phase not in ('idle', 'fault', 'finished'):
                response.success, response.message = False, '请先停止任务'
                return response
            self.flush_events()
            self.race = Race(timeout=args.timeout)
            self.session = uuid.uuid4().hex
            self.event_path = self.new_event_path()
            self.finish_ack, self.last_speech = False, 0.0
            response.success, response.message = True, '任务已复位'
            return response

        def update(self):
            now = time.monotonic()
            if self.event_writer and self.event_writer.status()['error']:
                self.fail_event_writer()
            state = self.race.tick(now)
            state.update({'session': self.session, 'calibrated': config['calibrated'],
                          'detections': self.last_detection, 'finish_announcement_ok': self.finish_ack})
            self.send(self.pub, state)
            # Requests are repeated until acknowledged; the player deduplicates them.
            if now - self.last_speech >= 0.5:
                if state['phase'] == 'yield' and not state['announcement_ok']:
                    self.send(self.speech, {'id': self.session + ':' + str(state['announcement_id']),
                                            'kind': 'crosswalk', 'text': '人行道前停车礼让行人'})
                    self.last_speech = now
                elif state['phase'] == 'finished' and not self.finish_ack:
                    self.send(self.speech, {'id': self.session + ':finish', 'kind': 'finish', 'text': '已完成两圈'})
                    self.last_speech = now
            self.flush_events()

    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush_events()
        if node.event_writer and not node.event_writer.close():
            node.get_logger().error('任务事件未能全部写入')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
