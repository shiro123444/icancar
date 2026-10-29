#!/usr/bin/env python3
"""ROS 1 command gate; publishes only when explicitly enabled."""
import argparse
import json
import sys
import threading
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.command import CommandGate


def main():
    p=argparse.ArgumentParser(description='ROS 1 速度仲裁接口')
    p.add_argument('--input-topic',required=True)
    p.add_argument('--output-topic',required=True)
    p.add_argument('--safety-topic',required=True)
    p.add_argument('--status-topic',default='/visual_race/status')
    p.add_argument('--max-linear',required=True,type=float)
    p.add_argument('--max-angular',required=True,type=float)
    p.add_argument('--timeout',type=float,default=.5)
    p.add_argument('--activate',action='store_true')
    args=p.parse_args()
    if args.input_topic.rstrip('/') == args.output_topic.rstrip('/'):p.error('输入和输出话题必须不同')
    gate=CommandGate(args.max_linear,args.max_angular,args.timeout,args.activate)
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String,Bool
    rospy.init_node('visual_race_command_gate')
    lock=threading.Lock()
    pub=rospy.Publisher(args.output_topic,Twist,queue_size=1) if args.activate else None
    def command(msg):
        with lock:gate.set_command(msg.linear.x,msg.linear.y,msg.angular.z,time.monotonic())
    def state(msg):
        try: value=json.loads(msg.data)
        except (TypeError,ValueError): value=None
        with lock:gate.set_state(value,time.monotonic())
    def safety(msg):
        with lock:gate.set_safety(msg.data,time.monotonic())
    def update(event):
        with lock: output=gate.output(time.monotonic())
        if pub is not None and output is not None:
            msg=Twist()
            msg.linear.x,msg.linear.y,msg.angular.z=output
            pub.publish(msg)
    def shutdown():
        if pub is not None:pub.publish(Twist())
    rospy.Subscriber(args.input_topic,Twist,command,queue_size=1)
    rospy.Subscriber(args.status_topic,String,state,queue_size=1)
    rospy.Subscriber(args.safety_topic,Bool,safety,queue_size=1)
    rospy.Timer(rospy.Duration(.1),update)
    rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__=='__main__':main()
