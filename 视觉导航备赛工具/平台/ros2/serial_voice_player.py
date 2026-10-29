#!/usr/bin/env python3
"""ROS 2 adapter for the verified vendor serial voice box."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.voice_protocol import VoiceBox


def main():
    p=argparse.ArgumentParser(description='ROS 2 语音盒固定停车提示')
    p.add_argument('--device',required=True)
    p.add_argument('--speech-topic',default='/visual_race/speech')
    p.add_argument('--ack-topic',default='/visual_race/speech_ack')
    args=p.parse_args()
    try:
        import serial
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as e:
        raise SystemExit('需要 ROS 2 与 pyserial：'+str(e))
    port=serial.Serial(args.device,9600,timeout=.4,write_timeout=1)
    box=VoiceBox(port)
    try: version=box.handshake()
    except Exception:
        port.close()
        raise
    class Player(Node):
        def __init__(self):
            super().__init__('visual_race_serial_voice')
            self.pub=self.create_publisher(String,args.ack_topic,10)
            self.create_subscription(String,args.speech_topic,self.request,10)
            self.completed=set()
            self.get_logger().info('已连接语音盒，固件版本 %.1f' % version)
        def ack(self,identifier,success):
            msg=String()
            msg.data=json.dumps({'id':identifier,'success':success,'transport':'serial',
                                 'meaning':'command_written','content_verified':False})
            self.pub.publish(msg)
        def request(self,msg):
            try:
                data=json.loads(msg.data)
                identifier,kind=data['id'],data['kind']
                if not isinstance(identifier,str) or not 1<=len(identifier)<=128: return
            except (ValueError,KeyError,TypeError): return
            if kind!='crosswalk':
                return
            if identifier not in self.completed:
                try:
                    box.play_stop()
                    self.completed.add(identifier)
                    self.get_logger().info('固定停车提示指令已发送')
                except (OSError,serial.SerialException) as e:
                    self.get_logger().error('语音指令发送失败: '+str(e))
                    self.ack(identifier,False)
                    return
            self.ack(identifier,True)
    rclpy.init()
    node=Player()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
