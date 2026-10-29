#!/usr/bin/env python3
"""ROS 1 camera/odometry adapter. Does not publish motion commands."""
import argparse
import json
import math
import sys
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from automation.detector import detect, validate_config
from automation.race import Race
from ros2.automatic_race import decode


def source_fresh(header, now_ns, max_age):
    try:
        stamp = header.stamp.to_nsec()
        age = (now_ns - stamp) / 1_000_000_000
        return stamp > 0 and -0.2 <= age <= max_age
    except (AttributeError, TypeError, ValueError):
        return False


def main():
    parser = argparse.ArgumentParser(description='ROS 1 自动任务，默认只读车辆')
    parser.add_argument('--camera-topic', required=True)
    parser.add_argument('--camera-type', required=True, choices=('raw', 'compressed'))
    parser.add_argument('--odom-topic', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--timeout', type=float, default=.6)
    parser.add_argument('--max-source-age', type=float, default=.8)
    parser.add_argument('--events-dir', default='')
    args = parser.parse_args()
    if not math.isfinite(args.max_source_age) or args.max_source_age <= 0:
        parser.error('max-source-age 必须是正有限数')
    with open(args.config, encoding='utf-8') as f:
        config = validate_config(json.load(f))
    import rospy
    from sensor_msgs.msg import Image, CompressedImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('visual_race_controller', anonymous=False)
    class Controller:
        def __init__(self):
            self.lock = threading.RLock()
            self.race, self.session = Race(timeout=args.timeout), uuid.uuid4().hex
            self.last_detection, self.finish_ack, self.last_speech = None, False, 0.0
            self.event_dir = Path(args.events_dir).expanduser() if args.events_dir else None
            if self.event_dir:
                self.event_dir.mkdir(parents=True, exist_ok=True)
            self.status_pub = rospy.Publisher('/visual_race/status', String, queue_size=1)
            self.speech_pub = rospy.Publisher('/visual_race/speech', String, queue_size=1)
            rospy.Subscriber(args.camera_topic, CompressedImage if args.camera_type=='compressed' else Image,
                             self.camera, queue_size=1, buff_size=2**24)
            rospy.Subscriber(args.odom_topic, Odometry, self.odom, queue_size=1)
            rospy.Subscriber('/visual_race/speech_ack', String, self.ack, queue_size=1)
            rospy.Service('/visual_race/arm', Trigger, self.arm)
            rospy.Service('/visual_race/stop', Trigger, self.stop)
            rospy.Service('/visual_race/reset', Trigger, self.reset)
            rospy.Timer(rospy.Duration(.1), self.update)
        def flush(self):
            for event in self.race.pop_events():
                line = json.dumps(dict(event, session=self.session, wall_time=datetime.now(timezone.utc).isoformat()),ensure_ascii=False)
                rospy.loginfo(line)
                if self.event_dir:
                    try:
                        with (self.event_dir / ('visual-race-' + self.session + '.jsonl')).open('a',encoding='utf-8') as f:
                            f.write(line + '\n')
                    except OSError as e: rospy.logerr('任务事件保存失败: %s', e)
        def camera(self, msg):
            if not source_fresh(msg.header, rospy.Time.now().to_nsec(), args.max_source_age):
                rospy.logwarn_throttle(3,'图像源时间戳过期或无效')
                return
            try:
                result = detect(decode(msg, args.camera_type), config)
                with self.lock:
                    self.last_detection = result
                    self.race.frame(time.monotonic(), result['start_line']['detected'], result['crosswalk']['detected'])
            except Exception as e: rospy.logwarn_throttle(3, '图像识别失败: %s', e)
        def odom(self, msg):
            if not source_fresh(msg.header, rospy.Time.now().to_nsec(), args.max_source_age):
                rospy.logwarn_throttle(3,'里程计源时间戳过期或无效')
                return
            try:
                v = msg.twist.twist
                with self.lock: self.race.odom(time.monotonic(), v.linear.x, v.linear.y, v.angular.z)
            except ValueError as e: rospy.logwarn_throttle(3, '里程计无效: %s', e)
        def ack(self, msg):
            try:
                data = json.loads(msg.data)
                with self.lock:
                    if data.get('id') == self.session + ':finish' and data.get('success') is True:
                        self.finish_ack = True
                    if data.get('id') == self.session + ':' + str(self.race.announcement_id):
                        self.race.acknowledge(time.monotonic(), self.race.announcement_id, data.get('success'))
            except (ValueError, AttributeError): pass
        def arm(self, request):
            with self.lock:
                try:
                    if not config['calibrated']: raise ValueError('先用现场图像标定配置')
                    self.race.arm(time.monotonic())
                    return TriggerResponse(True, '任务已准备')
                except ValueError as e: return TriggerResponse(False, str(e))
        def stop(self, request):
            with self.lock:
                self.race.phase, self.race.reason = 'fault', 'manual_stop'
                self.race.events.append({'type':'fault','reason':'manual_stop','at':time.monotonic()})
            return TriggerResponse(True, '任务已停止')
        def reset(self, request):
            with self.lock:
                if self.race.phase not in ('idle','fault','finished'):
                    return TriggerResponse(False, '请先停止任务')
                self.flush()
                self.race, self.session = Race(timeout=args.timeout), uuid.uuid4().hex
                self.finish_ack, self.last_speech = False, 0.0
            return TriggerResponse(True, '任务已复位')
        def update(self, event):
            with self.lock:
                now = time.monotonic()
                state = self.race.tick(now)
                state.update({'session':self.session,'calibrated':config['calibrated'],
                              'detections':self.last_detection,'finish_announcement_ok':self.finish_ack})
                self.status_pub.publish(String(data=json.dumps(state,ensure_ascii=False,allow_nan=False)))
                if now - self.last_speech >= .5:
                    if state['phase']=='yield' and not state['announcement_ok']:
                        self.speech_pub.publish(String(data=json.dumps({'id':self.session+':'+str(state['announcement_id']),
                            'kind':'crosswalk','text':'人行道前停车礼让行人'},ensure_ascii=False)))
                        self.last_speech = now
                    elif state['phase']=='finished' and not self.finish_ack:
                        self.speech_pub.publish(String(data=json.dumps({'id':self.session+':finish',
                            'kind':'finish','text':'已完成两圈'},ensure_ascii=False)))
                        self.last_speech = now
                self.flush()
    controller = Controller()
    rospy.spin()
    with controller.lock: controller.flush()


if __name__ == '__main__': main()
