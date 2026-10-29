#!/usr/bin/env python3
"""ROS 1 camera, odometry and task status bridge; publishes no vehicle commands."""
import argparse
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2.telemetry_bridge import Store, handler_for, parse_mission, yaw_from_quaternion
import math


def main():
    parser = argparse.ArgumentParser(description='ROS 1 只读相机、里程计及任务状态桥')
    parser.add_argument('--port', type=int, default=8770)
    parser.add_argument('--camera-topic', required=True, help='JPEG CompressedImage 话题')
    parser.add_argument('--odom-topic', default='')
    parser.add_argument('--mission-topic', default='')
    args = parser.parse_args()
    import rospy
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    store = Store(mode='ros1-readonly')
    rospy.init_node('visual_race_readonly_bridge', anonymous=True, disable_signals=True)
    def camera(msg):
        data = bytes(msg.data)
        if data.startswith(b'\xff\xd8'):
            store.set_frame(data)
    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if all(math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w)):
            store.set_pose({'x': p.x, 'y': p.y, 'yaw_rad': yaw_from_quaternion(q.x,q.y,q.z,q.w),
                            'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id})
    def mission(msg):
        try: store.set_mission(parse_mission(msg.data))
        except (ValueError, TypeError): pass
    rospy.Subscriber(args.camera_topic, CompressedImage, camera, queue_size=1, buff_size=2**24)
    if args.odom_topic:
        rospy.Subscriber(args.odom_topic, Odometry, odom, queue_size=1)
    if args.mission_topic:
        rospy.Subscriber(args.mission_topic, String, mission, queue_size=1)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(store))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo('只读桥运行在 127.0.0.1:%d', args.port)
    try: rospy.spin()
    except KeyboardInterrupt: pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == '__main__': main()
