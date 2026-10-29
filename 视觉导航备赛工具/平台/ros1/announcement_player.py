#!/usr/bin/env python3
"""ROS 1 audio player, deduplicating requests and acknowledging playback."""
import argparse
import json
import os
import subprocess
import threading
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description='ROS 1 人行道与完赛提示音')
    p.add_argument('--player', required=True, help='aplay 的绝对路径')
    p.add_argument('--device', default='', help='aplay 设备，例如 plughw:2,0')
    p.add_argument('--crosswalk-wav', required=True)
    p.add_argument('--finish-wav', required=True)
    p.add_argument('--timeout', type=float, default=15.0)
    args = p.parse_args()
    files = {'crosswalk':Path(args.crosswalk_wav),'finish':Path(args.finish_wav)}
    if not Path(args.player).is_absolute() or not os.access(args.player,os.X_OK): p.error('播放程序不可执行')
    if any(not file.is_file() for file in files.values()) or not 0 < args.timeout < 3600: p.error('音频文件或超时无效')
    import rospy
    from std_msgs.msg import String
    rospy.init_node('visual_race_speech_player', anonymous=False)
    class Player:
        def __init__(self):
            self.lock = threading.Lock()
            self.pub = rospy.Publisher('/visual_race/speech_ack',String,queue_size=10)
            self.process, self.identifier, self.started = None, None, None
            self.successes = set()
            rospy.Subscriber('/visual_race/speech',String,self.request,queue_size=10)
            rospy.Timer(rospy.Duration(.1),self.poll)
        def ack(self, identifier, success):
            self.pub.publish(String(data=json.dumps({'id':identifier,'success':success})))
        def request(self,msg):
            try:
                data = json.loads(msg.data)
                identifier,kind = data['id'],data['kind']
                if not isinstance(identifier,str) or not 1 <= len(identifier) <= 128 or kind not in files: return
            except (ValueError,KeyError,TypeError): return
            with self.lock:
                if identifier in self.successes: self.ack(identifier,True)
                elif self.process is None:
                    try:
                        cmd = [args.player] + (['-D',args.device] if args.device else []) + [str(files[kind])]
                        self.process = subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                        self.identifier,self.started = identifier,time.monotonic()
                    except OSError as e:
                        rospy.logerr('播放程序启动失败: %s', e)
                        self.ack(identifier,False)
        def poll(self,event):
            with self.lock:
                if self.process is None: return
                code=self.process.poll()
                if code is None and time.monotonic()-self.started > args.timeout:
                    self.process.kill()
                    code=self.process.wait()
                if code is not None:
                    success = code == 0
                    if success: self.successes.add(self.identifier)
                    self.ack(self.identifier,success)
                    rospy.loginfo('提示音播放%s', '完成' if success else '失败')
                    self.process,self.identifier=None,None
        def close(self):
            with self.lock:
                if self.process is not None:
                    self.process.terminate()
                    try:self.process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
    player=Player()
    rospy.on_shutdown(player.close)
    rospy.spin()


if __name__=='__main__':main()
