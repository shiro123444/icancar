#!/usr/bin/env python3
"""Send the vendor's fixed stop prompt to the verified serial voice box."""
import argparse
import json
import sys
import threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.voice_protocol import VoiceBox


def main():
    p=argparse.ArgumentParser(description='通过厂商语音盒播放固定停车提示')
    p.add_argument('--device',required=True,help='语音盒的已确认串口路径')
    args=p.parse_args()
    import serial
    import rospy
    from std_msgs.msg import String
    rospy.init_node('visual_race_serial_voice',anonymous=False)
    port=serial.Serial(args.device,9600,timeout=.4,write_timeout=1)
    box=VoiceBox(port)
    try:
        version=box.handshake()
    except Exception:
        port.close()
        raise
    rospy.loginfo('已连接语音盒，固件版本 %.1f',version)
    pub=rospy.Publisher('/visual_race/speech_ack',String,queue_size=10)
    lock,completed=threading.Lock(),set()
    def request(msg):
        try:
            data=json.loads(msg.data)
            identifier,kind=data['id'],data['kind']
            if not isinstance(identifier,str) or not 1<=len(identifier)<=128: return
        except (ValueError,KeyError,TypeError): return
        if kind!='crosswalk':
            rospy.logwarn_throttle(10,'完赛语音尚无已确认的设备指令')
            return
        with lock:
            if identifier not in completed:
                try:
                    box.play_stop()
                    completed.add(identifier)
                    rospy.loginfo('固定停车提示指令已发送')
                except (OSError,serial.SerialException) as e:
                    rospy.logerr('语音指令发送失败: %s',e)
                    pub.publish(String(data=json.dumps({'id':identifier,'success':False,'transport':'serial'})))
                    return
            pub.publish(String(data=json.dumps({'id':identifier,'success':True,'transport':'serial',
                                                 'meaning':'command_written','content_verified':False})))
    rospy.Subscriber('/visual_race/speech',String,request,queue_size=10)
    rospy.on_shutdown(port.close)
    rospy.spin()


if __name__=='__main__':main()
