#!/usr/bin/env python3
"""Run the manufacturer's race model on ROS 2 images without vehicle control."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.realtime import LatestWorker
from automation.steering_candidate import fresh_frame
from automation.yolo import predictions


def main():
    p=argparse.ArgumentParser(description='ROS 2 原厂视觉模型只读推理')
    p.add_argument('--model',required=True,help='race.xml 的绝对路径')
    p.add_argument('--camera-topic',default='/camera/rgb/image_raw/compressed')
    p.add_argument('--output-topic',default='/visual_race/model_detections')
    p.add_argument('--max-fps',type=float,default=5.)
    p.add_argument('--threshold',type=float,default=.30)
    p.add_argument('--threads',type=int,default=2)
    p.add_argument('--max-source-age',type=float,default=.8)
    p.add_argument('--timeout',type=float,default=.8)
    args=p.parse_args()
    model=Path(args.model)
    if not model.is_absolute() or not model.is_file() or not model.with_suffix('.bin').is_file():p.error('模型 XML 或 BIN 不存在')
    if (not math.isfinite(args.max_fps) or not 0<args.max_fps<=10 or not 0<args.threshold<1
            or not 1<=args.threads<=8 or not math.isfinite(args.max_source_age) or args.max_source_age<=0
            or not math.isfinite(args.timeout) or args.timeout<=0):p.error('推理参数无效')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.qos import QoSProfile,HistoryPolicy,ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from openvino.runtime import Core
    core=Core()
    network=core.read_model(str(model),str(model.with_suffix('.bin')))
    if tuple(network.input(0).shape)!=(1,3,640,640):raise SystemExit('模型输入尺寸与 race 模型不匹配')
    compiled=core.compile_model(network,'CPU',{'INFERENCE_NUM_THREADS':str(args.threads)})
    class Preview(Node):
        def __init__(self):
            super().__init__('visual_race_openvino_preview')
            self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
            self.pub=self.create_publisher(String,args.output_topic,1)
            self.create_subscription(CompressedImage,args.camera_topic,self.frame,
                QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.last_stamp_ns=0
            self.reject_through_ns=0
            self.last_good=None
            self.worker=LatestWorker(self.process,args.max_fps)
            self.worker.start()
            self.create_timer(.02,self.publish_result,clock=self.steady_clock)
            self.create_timer(.2,self.check_timeout,clock=self.steady_clock)
            self.get_logger().info('原厂视觉模型已加载，输出可供标定与决策使用')
        def send(self,value):
            message=String()
            message.data=json.dumps(value,ensure_ascii=False,allow_nan=False)
            self.pub.publish(message)
        def frame(self,msg):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if stamp<=self.last_stamp_ns or not fresh_frame(stamp,self.get_clock().now().nanoseconds,args.max_source_age):return
            self.last_stamp_ns=stamp
            if not 0<len(msg.data)<=10_000_000 or bytes(msg.data[:2])!=b'\xff\xd8':
                self.reject_through_ns=stamp
                self.last_good=None
                self.send({'source':'openvino_race','valid':False})
                return
            self.worker.submit((stamp,bytes(msg.data)))
        @staticmethod
        def process(frame):
            stamp,data=frame
            started=time.monotonic()
            if not data.startswith(b'\xff\xd8'):raise ValueError('无效的 JPEG 图像')
            image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.size==0 or image.shape[0]*image.shape[1]>16_000_000:raise ValueError('图像读取失败')
            h,w=image.shape[:2]
            rgb=cv2.cvtColor(cv2.resize(image,(640,640)),cv2.COLOR_BGR2RGB)
            tensor=np.ascontiguousarray(rgb.transpose(2,0,1)[None],dtype=np.float32)/255.0
            begin=time.monotonic()
            output=compiled([tensor])[compiled.output(0)]
            return {'source':'openvino_race','valid':True,'stamp_ns':stamp,'width':w,'height':h,
                    'inference_ms':round((time.monotonic()-begin)*1000,1),
                    'processing_ms':round((time.monotonic()-started)*1000,1),
                    'detections':predictions(output,w,h,args.threshold)}
        def publish_result(self):
            result=self.worker.take()
            if result is None:return
            if not result['ok']:
                self.last_good=None
                self.send({'source':'openvino_race','valid':False})
                self.get_logger().warning('模型推理失败: '+result['error'],throttle_duration_sec=3.0)
                return
            value=result['value']
            if (value['stamp_ns']<=self.reject_through_ns
                    or not fresh_frame(value['stamp_ns'],self.get_clock().now().nanoseconds,args.max_source_age)):
                self.send({'source':'openvino_race','valid':False})
                return
            self.last_good=time.monotonic()
            value['frames_replaced']=self.worker.stats()['input_replaced']
            self.send(value)
        def check_timeout(self):
            if self.last_good is None or time.monotonic()-self.last_good>args.timeout:
                self.send({'source':'openvino_race','valid':False})
    rclpy.init()
    node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.worker.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
