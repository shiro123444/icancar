#!/usr/bin/env python3
"""ROS 2 USB camera publisher, independent of ROS 1."""
import argparse
import math
import time


def read_calibration(data):
    if not isinstance(data,dict):raise ValueError('标定数据必须是字典')
    width,height=data.get('image_width'),data.get('image_height')
    if not isinstance(width,int) or not isinstance(height,int) or width<=0 or height<=0:
        raise ValueError('标定图像尺寸无效')
    result={'width':width,'height':height,'distortion_model':data.get('distortion_model','plumb_bob')}
    if not isinstance(result['distortion_model'],str) or not result['distortion_model']:
        raise ValueError('畸变模型无效')
    for source,target,count in (('camera_matrix','k',9),('distortion_coefficients','d',5),
                                ('rectification_matrix','r',9),('projection_matrix','p',12)):
        item=data.get(source)
        if not isinstance(item,dict):raise ValueError(source+' 标定数据无效')
        value=item.get('data')
        if not isinstance(value,list) or len(value)!=count or any(
                not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) for v in value):
            raise ValueError(source+' 标定数据无效')
        result[target]=[float(v) for v in value]
    if result['k'][0]<=0 or result['k'][4]<=0:raise ValueError('焦距无效')
    return result


def main():
    p=argparse.ArgumentParser(description='ROS 2 USB 相机')
    p.add_argument('--device',required=True)
    p.add_argument('--topic',default='/camera/rgb/image_raw/compressed')
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=480)
    p.add_argument('--fps',type=float,default=15)
    p.add_argument('--calibration',default='',help='已检查的相机内参 YAML')
    p.add_argument('--camera-info-topic',default='/camera/rgb/camera_info')
    args=p.parse_args()
    if not (0 < args.width <= 4096 and 0 < args.height <= 4096 and 0 < args.fps <= 60):
        p.error('相机分辨率或帧率无效')
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, CameraInfo
    calibration=None
    if args.calibration:
        import yaml
        with open(args.calibration,encoding='utf-8') as f:calibration=read_calibration(yaml.safe_load(f))
    cap=cv2.VideoCapture(args.device,cv2.CAP_V4L2)
    if not cap.isOpened():raise SystemExit('无法打开相机: '+args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,args.height)
    cap.set(cv2.CAP_PROP_FPS,args.fps)
    # V4L2 support varies; a False result means the device may still buffer.
    try:
        small_buffer=bool(cap.set(cv2.CAP_PROP_BUFFERSIZE,1)) if hasattr(cv2,'CAP_PROP_BUFFERSIZE') else False
    except cv2.error:
        small_buffer=False
    class Camera(Node):
        def __init__(self):
            super().__init__('visual_race_usb_camera')
            self.pub=self.create_publisher(CompressedImage,args.topic,qos_profile_sensor_data)
            self.info_pub=self.create_publisher(CameraInfo,args.camera_info_topic,qos_profile_sensor_data) if calibration else None
            self.create_timer(1/args.fps,self.frame)
            self.last_warning=0.0
            self.get_logger().info('相机单帧缓冲设置：'+('已接受' if small_buffer else '未确认'))
        def frame(self):
            # This is the start of the read, not a hardware exposure timestamp.
            # It includes read and JPEG encoding in downstream age checks.
            read_started=self.get_clock().now().to_msg()
            ok,frame=cap.read()
            if not ok:
                if time.monotonic()-self.last_warning>3:
                    self.get_logger().warning('相机没有返回图像')
                    self.last_warning=time.monotonic()
                return
            ok,encoded=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),85])
            if not ok:return
            msg=CompressedImage()
            msg.header.stamp=read_started
            msg.header.frame_id='camera'
            msg.format='jpeg'
            msg.data=encoded.tobytes()
            self.pub.publish(msg)
            if self.info_pub is not None and frame.shape[1]==calibration['width'] and frame.shape[0]==calibration['height']:
                info=CameraInfo()
                info.header=msg.header
                info.width,info.height=calibration['width'],calibration['height']
                info.distortion_model=calibration['distortion_model']
                info.k,info.d,info.r,info.p=calibration['k'],calibration['d'],calibration['r'],calibration['p']
                self.info_pub.publish(info)
    rclpy.init()
    node=Camera()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
