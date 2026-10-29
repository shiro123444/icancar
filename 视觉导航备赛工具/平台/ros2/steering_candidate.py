#!/usr/bin/env python3
"""Optional candidate velocity for the safety gate, never a direct base driver."""
import argparse
import json
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.steering_candidate import SteeringCandidate


def main():
    p = argparse.ArgumentParser(description='将循迹预览转换为限幅候选速度，默认不发布')
    p.add_argument('--input-topic', default='/visual_race/steering_preview')
    p.add_argument('--output-topic', default='/visual_race/candidate_vel')
    p.add_argument('--speed', type=float, default=.05)
    p.add_argument('--max-angular', type=float, default=.4)
    p.add_argument('--timeout', type=float, default=.5)
    p.add_argument('--max-frame-age', type=float, default=.4, help='输出时相机图像允许的最大年龄（秒）')
    p.add_argument('--window-ms', type=float, default=250, help='转向短窗口长度（毫秒，0 表示不平滑）')
    p.add_argument('--window-samples', type=int, default=3, help='短窗口最多保留的预测数（1–5）')
    p.add_argument('--hz', type=float, default=10.0, help='连续候选指令输出频率（1–20 Hz）')
    p.add_argument('--output-type', choices=('twist-stamped', 'twist'), default='twist-stamped',
                   help='带时间戳的候选速度，或兼容旧程序的无时间戳速度')
    p.add_argument('--sign', type=int, choices=(-1, 1), default=1)
    p.add_argument('--activate', action='store_true', help='发布候选速度，仍需要安全仲裁才可接入底盘')
    args = p.parse_args()
    if not args.output_topic.startswith('/visual_race/') or args.output_topic.rstrip('/') == args.input_topic.rstrip('/'):
        p.error('候选速度话题必须位于 /visual_race/ 下且不同于输入话题')
    if args.speed > .15 or args.max_angular > 1:
        p.error('候选速度上限超出低速调试范围')
    if not 1 <= args.hz <= 20:
        p.error('输出频率须为 1–20 Hz')
    try:
        candidate = SteeringCandidate(args.speed, args.max_angular, args.timeout, args.sign, args.activate,
                                      args.max_frame_age, args.window_ms / 1000, args.window_samples)
    except ValueError as error:
        p.error(str(error))
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist, TwistStamped
        from std_msgs.msg import String
    except ImportError as error:
        raise SystemExit('需要 ROS 2 环境：' + str(error))

    class CandidateNode(Node):
        def __init__(self):
            super().__init__('visual_race_steering_candidate')
            message_type = TwistStamped if args.output_type == 'twist-stamped' else Twist
            self.pub = self.create_publisher(message_type, args.output_topic, 1) if args.activate else None
            self.create_subscription(String, args.input_topic, self.on_preview, 1)
            self.create_timer(1 / args.hz, self.tick)
        def on_preview(self, msg):
            try:
                value = json.loads(msg.data, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('invalid JSON')))
            except (ValueError, TypeError):
                value = None
            candidate.update(value, time.monotonic(), self.get_clock().now().nanoseconds)
        def publish(self, values):
            if self.pub is None or values is None:
                return
            if args.output_type == 'twist-stamped':
                message = TwistStamped()
                if candidate.frame_stamp_ns is not None:
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(candidate.frame_stamp_ns, 1_000_000_000)
                message.header.frame_id = 'base_link'
                twist = message.twist
            else:
                message, twist = Twist(), None
                twist = message
            twist.linear.x, twist.linear.y, twist.angular.z = values
            self.pub.publish(message)
        def tick(self):
            self.publish(candidate.output(time.monotonic(), self.get_clock().now().nanoseconds))

    rclpy.init()
    node = CandidateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish((0.0, 0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
