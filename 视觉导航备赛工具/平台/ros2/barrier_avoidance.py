#!/usr/bin/env python3
"""Combine lane candidate and barrier observation into another gated candidate."""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.barrier_avoidance import BarrierAvoider, ZERO


def main():
    p = argparse.ArgumentParser(description='挡板绕行候选速度，默认不发布')
    p.add_argument('--config', required=True)
    p.add_argument('--lane-topic', default='/visual_race/candidate_vel')
    p.add_argument('--observation-topic', default='/visual_race/barrier_observation')
    p.add_argument('--output-topic', default='/visual_race/avoidance_candidate_vel')
    p.add_argument('--diagnostics-topic', default='/visual_race/avoidance_diagnostics')
    p.add_argument('--activate', action='store_true')
    args = p.parse_args()
    if (not args.output_topic.startswith('/visual_race/')
            or args.output_topic.rstrip('/') == args.lane_topic.rstrip('/')):
        p.error('输出必须是 /visual_race/ 下不同于输入的候选话题')
    with open(args.config, encoding='utf-8') as handle:
        planner = BarrierAvoider(json.load(handle))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.clock import Clock, ClockType
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
    except ImportError as error:
        raise SystemExit('需要 ROS 2 消息包：' + str(error))

    class Avoidance(Node):
        def __init__(self):
            super().__init__('visual_race_barrier_avoidance')
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.publisher = self.create_publisher(TwistStamped, args.output_topic, 1) if args.activate else None
            self.diagnostics = self.create_publisher(String, args.diagnostics_topic, 1)
            self.create_subscription(TwistStamped, args.lane_topic, self.lane, 1)
            self.create_subscription(String, args.observation_topic, self.observation, 1)
            self.create_timer(.1, self.update, clock=self.steady_clock)

        def lane(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            v = msg.twist
            planner.set_lane(v.linear.x, v.linear.y, v.angular.z, stamp,
                             time.monotonic(), self.get_clock().now().nanoseconds)
            self.stop_if_needed()

        def observation(self, msg):
            try:
                if len(msg.data) > 100_000:
                    raise ValueError('observation too large')
                value = json.loads(msg.data, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('invalid JSON')))
            except (TypeError, ValueError):
                value = None
            planner.set_observation(value, time.monotonic(), self.get_clock().now().nanoseconds)
            self.stop_if_needed()

        def compute(self):
            return planner.output(time.monotonic(), self.get_clock().now().nanoseconds)

        def publish(self, values, stamp):
            if self.publisher is None:
                return
            msg = TwistStamped()
            if stamp is not None:
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = values
            self.publisher.publish(msg)

        def stop_if_needed(self):
            values, stamp, _ = self.compute()
            if values == ZERO:
                self.publish(values, stamp)

        def update(self):
            values, stamp, reason = self.compute()
            self.publish(values, stamp)
            self.diagnostics.publish(String(data=json.dumps({'source':'barrier_avoidance', 'reason':reason,
                                      'calibrated':planner.config['calibrated']}, ensure_ascii=False)))

    rclpy.init()
    node = Avoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(ZERO, None)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
