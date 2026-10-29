#!/usr/bin/env python3
"""Replay a saved image through candidate and gate in ROS 2 domain 78 only."""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description='隔离 ROS 2 域中的模型、候选速度和安全仲裁检查')
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--image', type=Path, required=True)
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '78' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('仅允许在本机隔离 ROS 2 域 78 中运行')
    root = Path(__file__).resolve().parents[1]
    model, image = args.model.resolve(), args.image.read_bytes()
    if not model.is_file() or not image.startswith(b'\xff\xd8'):
        raise SystemExit('模型或 JPEG 图片不存在')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    sys.path.insert(0, str(root))
    from automation.command import VelocityBuffer

    topics = {'image': '/visual_race/trial/image', 'preview': '/visual_race/trial/preview',
              'candidate': '/visual_race/trial/candidate', 'status': '/visual_race/trial/status',
              'safety': '/visual_race/trial/safety', 'output': '/visual_race/trial/output',
              'odom': '/visual_race/trial/odom', 'estop': '/visual_race/trial/estop',
              'reset': '/visual_race/trial/reset_estop', 'diagnostics': '/visual_race/trial/diagnostics'}
    commands = [
        [sys.executable, str(root/'ros2/steering_preview.py'), '--model', str(model), '--camera-topic', topics['image'], '--output-topic', topics['preview']],
        [sys.executable, str(root/'ros2/steering_candidate.py'), '--activate', '--input-topic', topics['preview'], '--output-topic', topics['candidate'], '--speed', '.05', '--max-angular', '.4'],
        [sys.executable, str(root/'ros2/command_gate.py'), '--activate', '--input-topic', topics['candidate'], '--output-topic', topics['output'], '--status-topic', topics['status'], '--safety-topic', topics['safety'], '--feedback-topic', topics['odom'], '--estop-topic', topics['estop'], '--reset-service', topics['reset'], '--diagnostics-topic', topics['diagnostics'], '--max-linear', '.05', '--max-angular', '.4']]
    with tempfile.TemporaryFile(mode='w+t') as log:
        children = [subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT) for cmd in commands]
        rclpy.init()
        class Trial(Node):
            def __init__(self):
                super().__init__('isolated_steering_gate_replay')
                self.image_pub = self.create_publisher(CompressedImage, topics['image'], qos_profile_sensor_data)
                self.preview_pub = self.create_publisher(String, topics['preview'], 10)
                self.state_pub = self.create_publisher(String, topics['status'], 10)
                self.safety_pub = self.create_publisher(Bool, topics['safety'], 10)
                self.estop_pub = self.create_publisher(Bool, topics['estop'], 10)
                self.odom_pub = self.create_publisher(Odometry, topics['odom'], qos_profile_sensor_data)
                self.reset_client = self.create_client(Trigger, topics['reset'])
                self.reset_future = None
                self.initial_reset_done = False
                self.base_buffer = VelocityBuffer(.05, .4, .3, .3)
                self.create_subscription(TwistStamped, topics['output'], self.on_output, 1)
                self.create_timer(.1, self.tick)
                self.stage, self.changed_at = 'warmup', time.monotonic()
                self.seen = {'forward': False, 'safety_stop': False, 'resumed': False,
                             'estop_stop': False, 'estop_held': False, 'estop_resumed': False,
                             'old_prediction_stop': False, 'fresh_resume': False, 'stale_stop': False}
            def tick(self):
                self.state_pub.publish(String(data=json.dumps({'phase':'running','action':'follow_lane','calibrated':True})))
                self.safety_pub.publish(Bool(data=self.stage != 'safety_stop'))
                self.estop_pub.publish(Bool(data=self.stage == 'estop_stop'))
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                self.odom_pub.publish(odom)
                if self.reset_future is not None and self.reset_future.done():
                    result = self.reset_future.result()
                    self.reset_future = None
                    if result.success:
                        if self.stage == 'warmup': self.initial_reset_done = True
                        elif self.stage == 'estop_clear': self.transition('after_estop_reset')
                wants_reset = ((self.stage == 'warmup' and not self.initial_reset_done) or
                               (self.stage == 'estop_clear' and self.seen['estop_held']))
                if wants_reset and self.reset_future is None and self.reset_client.service_is_ready():
                    self.reset_future = self.reset_client.call_async(Trigger.Request())
                if self.stage == 'old_prediction_stop':
                    old_stamp = self.get_clock().now().nanoseconds - 2_000_000_000
                    self.preview_pub.publish(String(data=json.dumps({'source':'steering_cnn','valid':True,
                                            'steering':.2,'inference_ms':1,'image_stamp_ns':old_stamp})))
                if self.stage != 'stale_stop':
                    msg = CompressedImage()
                    if self.stage == 'old_prediction_stop':
                        old_stamp = self.get_clock().now().nanoseconds - 2_000_000_000
                        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(old_stamp, 1_000_000_000)
                    else:
                        msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format, msg.data = 'jpeg', image
                    self.image_pub.publish(msg)
            def transition(self, name):
                self.stage, self.changed_at = name, time.monotonic()
            def on_output(self, msg):
                now = time.monotonic()
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                raw = msg.twist
                if (not all(math.isfinite(v) for v in (raw.linear.x, raw.linear.y, raw.angular.z))
                        or math.hypot(raw.linear.x, raw.linear.y) > .050001 or abs(raw.angular.z) > .400001):
                    raise ValueError('隔离测试仲裁输出超限')
                self.base_buffer.update(raw.linear.x, raw.linear.y, raw.angular.z,
                                        now, stamp, self.get_clock().now().nanoseconds)
                x, y, w = self.base_buffer.output(now, self.get_clock().now().nanoseconds)
                if not all(math.isfinite(v) for v in (x,y,w)) or math.hypot(x,y) > .050001 or abs(w) > .400001:
                    raise ValueError('隔离测试输出超限')
                moving = abs(x) > .001
                if self.stage == 'warmup' and moving:
                    self.seen['forward'] = True
                    self.transition('safety_stop')
                elif self.stage == 'safety_stop' and not moving and time.monotonic() - self.changed_at > .3:
                    self.seen['safety_stop'] = True
                    self.transition('resume')
                elif self.stage == 'resume' and moving:
                    self.seen['resumed'] = True
                    self.transition('estop_stop')
                elif self.stage == 'estop_stop' and not moving and time.monotonic() - self.changed_at > .3:
                    self.seen['estop_stop'] = True
                    self.transition('estop_clear')
                elif self.stage == 'estop_clear' and not moving and time.monotonic() - self.changed_at > .4:
                    self.seen['estop_held'] = True
                elif self.stage == 'after_estop_reset' and moving:
                    self.seen['estop_resumed'] = True
                    self.transition('old_prediction_stop')
                elif self.stage == 'old_prediction_stop' and not moving and time.monotonic() - self.changed_at > .6:
                    self.seen['old_prediction_stop'] = True
                    self.transition('fresh_resume')
                elif self.stage == 'fresh_resume' and moving:
                    self.seen['fresh_resume'] = True
                    self.transition('stale_stop')
                elif self.stage == 'stale_stop' and not moving and time.monotonic() - self.changed_at > .6:
                    self.seen['stale_stop'] = True
        node = None
        try:
            node = Trial()
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline and not node.seen['stale_stop'] and all(c.poll() is None for c in children):
                rclpy.spin_once(node, timeout_sec=.2)
            if not all(node.seen.values()):
                log.seek(0)
                raise SystemExit('隔离回放失败，阶段 ' + node.stage + ': ' + log.read()[-2500:])
            print(json.dumps({'checks': node.seen, 'domain':78, 'physical_motor_commands':False}, ensure_ascii=False))
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
            for child in children:
                child.terminate()
            for child in children:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()


if __name__ == '__main__':
    main()
