#!/usr/bin/env python3
"""Capture varied checkerboard views and compute ROS 2 camera intrinsics."""
import argparse
import json
import math
import time
from pathlib import Path


def signature(points,cols,rows,width,height):
    tl,tr,bl,br=points[0],points[cols-1],points[(rows-1)*cols],points[-1]
    def distance(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
    cx=sum(p[0] for p in points)/len(points)/width
    cy=sum(p[1] for p in points)/len(points)/height
    top,bottom=distance(tl,tr),distance(bl,br)
    left,right=distance(tl,bl),distance(tr,br)
    scale=math.sqrt(max(top*bottom*left*right,1e-12))/(width*height)
    angle=math.atan2(tr[1]-tl[1],tr[0]-tl[0])
    perspective=math.log(max(top,1e-6)/max(bottom,1e-6))+math.log(max(left,1e-6)/max(right,1e-6))
    return cx,cy,scale,angle,perspective


def novel(item,history):
    for old in history:
        shift=math.hypot(item[0]-old[0],item[1]-old[1])
        scale=abs(math.log(max(item[2],1e-8)/max(old[2],1e-8)))
        angle=abs(math.atan2(math.sin(item[3]-old[3]),math.cos(item[3]-old[3])))
        if shift<.055 and scale<.12 and angle<.12 and abs(item[4]-old[4])<.10:return False
    return True


def main():
    p=argparse.ArgumentParser(description='ROS 2 相机棋盘格内参标定')
    p.add_argument('--topic',default='/camera/rgb/image_raw/compressed')
    p.add_argument('--cols',type=int,default=8,help='棋盘格横向内角点数量')
    p.add_argument('--rows',type=int,default=6,help='棋盘格纵向内角点数量')
    p.add_argument('--square-size-m',type=float,required=True,help='实际测量的方格边长，单位米')
    p.add_argument('--samples',type=int,default=18)
    p.add_argument('--interval',type=float,default=.7)
    p.add_argument('--timeout',type=float,default=180)
    p.add_argument('--out',required=True,help='新建输出目录')
    args=p.parse_args()
    if not (3<=args.cols<=30 and 3<=args.rows<=30 and 12<=args.samples<=60
            and all(math.isfinite(x) and x>0 for x in (args.square_size_m,args.interval,args.timeout))):
        p.error('棋盘格尺寸或采样参数无效')
    out=Path(args.out).expanduser()
    out.mkdir(parents=True,exist_ok=False)
    import cv2
    import numpy as np
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    rclpy.init()
    class Collector(Node):
        def __init__(self):
            super().__init__('visual_race_camera_calibration')
            self.create_subscription(CompressedImage,args.topic,self.frame,qos_profile_sensor_data)
            self.last_attempt=0.0
            self.signatures=[]
            self.image_points=[]
            self.image_size=None
            self.last_board_at=None
            self.calibration_state='collecting'
            self.status_pub=self.create_publisher(String,'/visual_race/calibration',10)
            self.create_timer(.5,self.publish_status)
        def publish_status(self):
            visible=self.last_board_at is not None and time.monotonic()-self.last_board_at<1.5
            payload={'state':self.calibration_state,'accepted':len(self.image_points),
                     'target':args.samples,'board_visible':visible}
            msg=String()
            msg.data=json.dumps(payload,ensure_ascii=False)
            self.status_pub.publish(msg)
        def frame(self,msg):
            now=time.monotonic()
            if now-self.last_attempt<args.interval or len(self.image_points)>=args.samples:return
            self.last_attempt=now
            if (self.get_clock().now().nanoseconds-(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec))/1e9>1:
                return
            image=cv2.imdecode(np.frombuffer(bytes(msg.data),dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None:return
            h,w=image.shape[:2]
            if self.image_size is not None and self.image_size!=(w,h):return
            gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK
            ok,corners=cv2.findChessboardCorners(gray,(args.cols,args.rows),flags)
            if not ok:return
            self.last_board_at=now
            grid=corners.reshape(args.rows,args.cols,2)
            horizontal=np.linalg.norm(np.diff(grid,axis=1),axis=2).flatten()
            vertical=np.linalg.norm(np.diff(grid,axis=0),axis=2).flatten()
            spacing=float(np.median(np.concatenate((horizontal,vertical))))
            if spacing<8:return
            window=max(2,min(7,int(spacing*.22)))
            cv2.cornerSubPix(gray,corners,(window,window),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.001))
            flat=[(float(x),float(y)) for x,y in corners.reshape(-1,2)]
            item=signature(flat,args.cols,args.rows,w,h)
            if not novel(item,self.signatures):return
            self.image_size=(w,h)
            self.signatures.append(item)
            self.image_points.append(corners)
            path=out/('sample-%02d.jpg'%len(self.image_points))
            if not cv2.imwrite(str(path),image):raise RuntimeError('无法保存标定图片')
            print(json.dumps({'accepted':len(self.image_points),'needed':args.samples,
                              'center':[round(item[0],2),round(item[1],2)],'image':str(path)},ensure_ascii=False),flush=True)
    node=Collector()
    deadline=time.monotonic()+args.timeout
    print(json.dumps({'state':'collecting','needed':args.samples,'timeout_s':args.timeout},ensure_ascii=False),flush=True)
    try:
        while rclpy.ok() and len(node.image_points)<args.samples and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
        if len(node.image_points)<args.samples:
            raise SystemExit('采样超时：获得 %d/%d 张，已保留采集图片' % (len(node.image_points),args.samples))
        object_points=np.zeros((args.cols*args.rows,3),np.float32)
        object_points[:,:2]=np.mgrid[0:args.cols,0:args.rows].T.reshape(-1,2)*args.square_size_m
        rms,matrix,distortion,rvecs,tvecs=cv2.calibrateCamera(
            [object_points.copy() for _ in node.image_points],node.image_points,node.image_size,None,None)
        if not math.isfinite(float(rms)) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
            raise SystemExit('标定结果无效，请重新采集')
        width,height=node.image_size
        optimal,_=cv2.getOptimalNewCameraMatrix(matrix,distortion,(width,height),1,(width,height))
        projection=np.zeros((3,4),dtype=float)
        projection[:3,:3]=optimal
        per_view=[]
        for points,rvec,tvec in zip(node.image_points,rvecs,tvecs):
            projected,_=cv2.projectPoints(object_points,rvec,tvec,matrix,distortion)
            per_view.append(float(np.sqrt(np.mean(np.sum((points-projected)**2,axis=2)))))
        sigs=node.signatures
        report={'image_width':width,'image_height':height,'camera_name':'visual_race_camera',
                'distortion_model':'plumb_bob','camera_matrix':{'rows':3,'cols':3,'data':matrix.flatten().tolist()},
                'distortion_coefficients':{'rows':1,'cols':int(distortion.size),'data':distortion.flatten().tolist()},
                'rectification_matrix':{'rows':3,'cols':3,'data':np.eye(3).flatten().tolist()},
                'projection_matrix':{'rows':3,'cols':4,'data':projection.flatten().tolist()},
                'reprojection_rms_px':float(rms),'sample_count':len(node.image_points),
                'per_view_rms_px':per_view,
                'board':{'inner_corners':[args.cols,args.rows],'square_size_m':args.square_size_m},
                'center_span':[max(s[0] for s in sigs)-min(s[0] for s in sigs),
                               max(s[1] for s in sigs)-min(s[1] for s in sigs)]}
        with (out/'camera-intrinsics.yaml').open('x',encoding='utf-8') as f:
            yaml.safe_dump(report,f,sort_keys=False,allow_unicode=True)
        node.calibration_state='complete'
        node.publish_status()
        print(json.dumps({'state':'complete','output':str(out/'camera-intrinsics.yaml'),
                          'rms_px':round(float(rms),3),'center_span':report['center_span']},ensure_ascii=False),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
