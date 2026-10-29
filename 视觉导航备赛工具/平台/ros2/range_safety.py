#!/usr/bin/env python3
"""Publish safety permission from an independently verified ROS 2 Range sensor."""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.range_safety import RangeSafety


def main():
    p = argparse.ArgumentParser(description='距离传感器安全许可，默认配置不放行')
    p.add_argument('--config', required=True)
    p.add_argument('--range-topic', required=True, help='已核实的 sensor_msgs/msg/Range 话题')
    p.add_argument('--safety-topic', default='/visual_race/safety')
    args = p.parse_args()
    if not args.safety_topic.startswith('/visual_race/'):
        p.error('安全话题必须位于 /visual_race/ 下')
    with open(args.config, encoding='utf-8') as handle:
        guard = RangeSafety(json.load(handle))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Range
        from std_msgs.msg import Bool
    except ImportError as error:
        raise SystemExit('需要 ROS 2 和 Range 消息包：' + str(error))

    class Guard(Node):
        def __init__(self):
            super().__init__('visual_race_range_safety')
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.publisher = self.create_publisher(Bool, args.safety_topic, 1)
            self.create_subscription(Range, args.range_topic, self.range, qos_profile_sensor_data)
            self.create_timer(.1, self.publish, clock=self.steady_clock)
        def range(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            guard.update(msg.range, msg.min_range, msg.max_range, stamp,
                         time.monotonic(), self.get_clock().now().nanoseconds)
            if not guard.allowed(time.monotonic(), self.get_clock().now().nanoseconds):
                self.publisher.publish(Bool(data=False))
        def publish(self):
            self.publisher.publish(Bool(data=guard.allowed(time.monotonic(), self.get_clock().now().nanoseconds)))

    rclpy.init()
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
