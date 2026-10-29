#!/usr/bin/env python3
"""Optional ROS 2 velocity gate, disabled unless explicitly activated."""
import argparse
import json
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.command import CommandGate
from automation.feedback import EmergencyLatch, FeedbackGuard
from automation.supervisor import MotionSupervisor


def main():
    p = argparse.ArgumentParser(description='自动任务速度接口，默认不发布到车辆')
    p.add_argument('--input-topic', required=True, help='现有循迹程序的候选速度话题')
    p.add_argument('--output-topic', required=True, help='已核实的底盘速度话题')
    p.add_argument('--status-topic', default='/visual_race/status')
    p.add_argument('--safety-topic', required=True, help='独立安全监测的 Bool 话题，持续发送 true 才允许运动')
    p.add_argument('--max-linear', required=True, type=float)
    p.add_argument('--max-angular', required=True, type=float)
    p.add_argument('--timeout', type=float, default=0.5)
    p.add_argument('--input-type', choices=('twist-stamped', 'twist'), default='twist-stamped',
                   help='候选指令类型；无时间戳类型只适用于兼容旧程序')
    p.add_argument('--max-frame-age', type=float, default=.4, help='带时间戳的候选速度最大图像年龄')
    p.add_argument('--feedback-topic', default='/raw_odom', help='实测里程计话题')
    p.add_argument('--feedback-timeout', type=float, default=.5)
    p.add_argument('--feedback-max-age', type=float, default=.5)
    p.add_argument('--feedback-max-linear', type=float, default=.3)
    p.add_argument('--feedback-max-angular', type=float, default=1.2)
    p.add_argument('--stop-grace', type=float, default=1.0, help='发出停止后等待静止反馈的最长时间')
    p.add_argument('--estop-topic', default='/visual_race/estop', help='急停状态心跳，true 表示急停')
    p.add_argument('--reset-service', default='/visual_race/reset_estop', help='人工复位服务')
    p.add_argument('--diagnostics-topic', default='/visual_race/control_diagnostics')
    p.add_argument('--activate', action='store_true', help='明确启用底盘指令发布')
    args = p.parse_args()
    if args.input_topic.rstrip('/') == args.output_topic.rstrip('/'):
        p.error('输入、输出话题不能相同')
    try:
        gate = CommandGate(args.max_linear, args.max_angular, args.timeout, args.activate,
                           args.max_frame_age if args.input_type == 'twist-stamped' else None)
        feedback = FeedbackGuard(args.feedback_timeout, args.feedback_max_age, args.feedback_max_linear,
                                 args.feedback_max_angular, stop_grace=args.stop_grace)
        emergency = EmergencyLatch(args.timeout)
        supervisor = MotionSupervisor(gate, feedback, emergency)
    except ValueError as error:
        p.error(str(error))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import Twist, TwistStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String, Bool
        from std_srvs.srv import Trigger
    except ImportError as e:
        raise SystemExit('需要小车的 ROS 2 环境：' + str(e))

    class GateNode(Node):
        def __init__(self):
            super().__init__('visual_race_command_gate')
            self.publisher = self.create_publisher(Twist, args.output_topic, 1) if args.activate else None
            self.diagnostics = self.create_publisher(String, args.diagnostics_topic, 1)
            self.create_subscription(TwistStamped if args.input_type == 'twist-stamped' else Twist, args.input_topic, self.command, 1)
            self.create_subscription(String, args.status_topic, self.state, 1)
            self.create_subscription(Bool, args.safety_topic, self.safety, 1)
            self.create_subscription(Bool, args.estop_topic, self.estop, 1)
            sensor_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Odometry, args.feedback_topic, self.odometry, sensor_qos)
            self.create_service(Trigger, args.reset_service, self.reset)
            self.create_timer(0.1, self.update)
            self.create_timer(0.5, self.report)
        def command(self, msg):
            if args.input_type == 'twist-stamped':
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                twist = msg.twist
            else:
                stamp, twist = None, msg
            gate.set_command(twist.linear.x, twist.linear.y, twist.angular.z, time.monotonic(),
                             stamp, self.get_clock().now().nanoseconds)
            self.publish_stop_if_needed()
        def state(self, msg):
            try:
                value = json.loads(msg.data)
            except (TypeError, ValueError):
                value = None
            gate.set_state(value, time.monotonic())
            self.publish_stop_if_needed()
        def safety(self, msg):
            gate.set_safety(msg.data, time.monotonic())
            self.publish_stop_if_needed()
        def estop(self, msg):
            emergency.update(msg.data, time.monotonic())
            self.publish_stop_if_needed()
        def odometry(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            measured = msg.twist.twist
            feedback.update(measured.linear.x, measured.linear.y, measured.angular.z,
                            stamp, time.monotonic(), self.get_clock().now().nanoseconds)
            self.publish_stop_if_needed()
        def reset(self, request, response):
            now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
            response.success, response.message = supervisor.reset(now, ros_now)
            return response
        def compute(self):
            now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
            return supervisor.output(now, ros_now)
        def publish_stop_if_needed(self):
            values = self.compute()
            if values == (0.0, 0.0, 0.0):
                self.publish(values)
        def publish(self, values):
            if self.publisher is None or values is None:
                return
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = values
            self.publisher.publish(msg)
        def update(self):
            self.publish(self.compute())
        def report(self):
            now = time.monotonic()
            data = supervisor.diagnostics(now)
            self.diagnostics.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))

    rclpy.init()
    node = GateNode()
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
