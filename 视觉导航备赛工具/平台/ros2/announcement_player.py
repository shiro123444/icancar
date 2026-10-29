#!/usr/bin/env python3
"""Play prepared WAV prompts and acknowledge successful player exits."""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description='播放人行道与完赛提示音')
    p.add_argument('--player', required=True, help='小车上的音频播放程序绝对路径')
    p.add_argument('--crosswalk-wav', required=True)
    p.add_argument('--finish-wav', required=True)
    p.add_argument('--speech-topic', default='/visual_race/speech')
    p.add_argument('--ack-topic', default='/visual_race/speech_ack')
    p.add_argument('--timeout', type=float, default=15.0)
    args = p.parse_args()
    files = {'crosswalk': Path(args.crosswalk_wav), 'finish': Path(args.finish_wav)}
    if not Path(args.player).is_absolute() or not os.access(args.player, os.X_OK):
        p.error('player 必须是可执行文件的绝对路径')
    if any(not f.is_file() for f in files.values()) or args.timeout <= 0:
        p.error('音频文件或超时无效')
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as e:
        raise SystemExit('需要小车的 ROS 2 环境：' + str(e))

    class Player(Node):
        def __init__(self):
            super().__init__('visual_race_speech_player')
            self.pub = self.create_publisher(String, args.ack_topic, 10)
            self.create_subscription(String, args.speech_topic, self.request, 10)
            self.process, self.identifier, self.started = None, None, None
            self.successes = set()
            self.create_timer(0.1, self.poll)
        def acknowledge(self, identifier, success):
            msg = String()
            msg.data = json.dumps({'id': identifier, 'success': success})
            self.pub.publish(msg)
        def request(self, msg):
            try:
                data = json.loads(msg.data)
                identifier, kind = data['id'], data['kind']
                if not isinstance(identifier, str) or not 1 <= len(identifier) <= 128 or kind not in files:
                    return
            except (ValueError, KeyError, TypeError):
                return
            if identifier in self.successes:
                self.acknowledge(identifier, True)
            elif self.process is None:
                try:
                    self.process = subprocess.Popen([args.player, str(files[kind])], stdout=subprocess.DEVNULL,
                                                    stderr=subprocess.DEVNULL)
                    self.identifier, self.started = identifier, time.monotonic()
                except OSError as e:
                    self.get_logger().error('播放程序启动失败: ' + str(e))
                    self.acknowledge(identifier, False)
        def poll(self):
            if self.process is None:
                return
            code = self.process.poll()
            if code is None and time.monotonic() - self.started > args.timeout:
                self.process.kill()
                code = self.process.wait()
            if code is not None:
                success = code == 0
                if success:
                    self.successes.add(self.identifier)
                self.acknowledge(self.identifier, success)
                self.process, self.identifier = None, None
        def close_player(self):
            if self.process is not None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()

    rclpy.init()
    node = Player()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_player()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
