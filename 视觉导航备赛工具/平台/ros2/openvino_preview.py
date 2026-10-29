#!/usr/bin/env python3
"""Run the manufacturer's race model on ROS 2 images without vehicle control."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from automation.yolo import predictions


def main():
    p=argparse.ArgumentParser(description='ROS 2 原厂视觉模型只读推理')
    p.add_argument('--model',required=True,help='race.xml 的绝对路径')
    p.add_argument('--camera-topic',default='/camera/rgb/image_raw/compressed')
    p.add_argument('--output-topic',default='/visual_race/model_detections')
    p.add_argument('--max-fps',type=float,default=1.)
    p.add_argument('--threshold',type=float,default=.30)
    p.add_argument('--threads',type=int,default=2)
    args=p.parse_args()
    model=Path(args.model)
    if not model.is_absolute() or not model.is_file() or not model.with_suffix('.bin').is_file():p.error('模型 XML 或 BIN 不存在')
    if not math.isfinite(args.max_fps) or not 0<args.max_fps<=10 or not 0<args.threshold<1 or not 1<=args.threads<=8:p.error('推理参数无效')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
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
            self.pub=self.create_publisher(String,args.output_topic,10)
            self.create_subscription(CompressedImage,args.camera_topic,self.frame,
                QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.last=0.
            self.get_logger().info('原厂视觉模型已加载，输出仅供查看')
        def frame(self,msg):
            now=time.monotonic()
            if now-self.last<1/args.max_fps:return
            self.last=now
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if not stamp or (self.get_clock().now().nanoseconds-stamp)/1e9>1:return
            try:
                image=cv2.imdecode(np.frombuffer(bytes(msg.data),np.uint8),cv2.IMREAD_COLOR)
                if image is None:return
                h,w=image.shape[:2]
                rgb=cv2.cvtColor(cv2.resize(image,(640,640)),cv2.COLOR_BGR2RGB)
                tensor=np.ascontiguousarray(rgb.transpose(2,0,1)[None],dtype=np.float32)/255.0
                begin=time.monotonic()
                output=compiled([tensor])[compiled.output(0)]
                result={'source':'openvino_race','stamp_ns':stamp,'width':w,'height':h,
                        'inference_ms':round((time.monotonic()-begin)*1000,1),
                        'detections':predictions(output,w,h,args.threshold)}
                message=String()
                message.data=json.dumps(result,ensure_ascii=False,allow_nan=False)
                self.pub.publish(message)
            except Exception as e:self.get_logger().error('模型推理失败: '+str(e))
    rclpy.init()
    node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
