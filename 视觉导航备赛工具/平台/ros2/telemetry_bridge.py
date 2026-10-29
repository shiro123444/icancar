#!/usr/bin/env python3
"""Read-only ROS 2 camera/odometry bridge. Does not publish motion commands."""
import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

class Store:
    def __init__(self, clock=time.monotonic, mode='ros2-readonly'):
        self.lock = threading.Lock()
        self.clock = clock
        self.mode = mode
        self.pose = None
        self.odom_at = None
        self.frame = None
        self.frame_at = None
        self.graph = []
        self.mission = None
        self.mission_at = None
        self.model = None
        self.model_at = None
        self.calibration = None
        self.calibration_at = None
        self.steering = None
        self.steering_at = None
        self.control = None
        self.control_at = None
    def set_pose(self, pose):
        with self.lock:
            self.pose = pose
            self.odom_at = self.clock()
    def set_frame(self, data):
        if len(data) > 10_000_000: return
        with self.lock:
            self.frame = bytes(data)
            self.frame_at = self.clock()
    def set_graph(self, graph):
        with self.lock: self.graph = graph
    def set_mission(self, value):
        with self.lock:
            self.mission = value
            self.mission_at = self.clock()
    def set_model(self, value):
        with self.lock:
            self.model = value
            self.model_at = self.clock()
    def set_calibration(self, value):
        with self.lock:
            self.calibration = value
            self.calibration_at = self.clock()
    def set_steering(self, value):
        with self.lock:
            self.steering = value
            self.steering_at = self.clock()
    def set_control(self, value):
        with self.lock:
            self.control = value
            self.control_at = self.clock()
    def status(self):
        with self.lock:
            now = self.clock()
            return {'mode':self.mode, 'pose':self.pose,
                    'odom_age_s':None if self.odom_at is None else round(now-self.odom_at,2),
                    'camera_age_s':None if self.frame_at is None else round(now-self.frame_at,2),
                    'topics':self.graph, 'mission':self.mission,
                    'mission_age_s':None if self.mission_at is None else round(now-self.mission_at,2),
                    'model':self.model,'model_age_s':None if self.model_at is None else round(now-self.model_at,2),
                    'calibration':self.calibration,'calibration_age_s':None if self.calibration_at is None else round(now-self.calibration_at,2),
                    'steering':self.steering,'steering_age_s':None if self.steering_at is None else round(now-self.steering_at,2),
                    'control':self.control,'control_age_s':None if self.control_at is None else round(now-self.control_at,2)}
    def image(self):
        with self.lock:
            if self.frame_at is None or self.clock() - self.frame_at > 3: return None
            return self.frame

def yaw_from_quaternion(x,y,z,w):
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

def parse_mission(raw):
    if len(raw) > 100_000: raise ValueError('mission message too large')
    def reject_constant(value): raise ValueError('non-finite JSON value')
    value=json.loads(raw,parse_constant=reject_constant)
    if not isinstance(value,dict) or value.get('phase') not in ('idle','armed','running','yield','finished','fault'):
        raise ValueError('invalid mission state')
    for name in ('laps','crosswalk_stops'):
        x=value.get(name)
        if not isinstance(x,int) or isinstance(x,bool) or x<0: raise ValueError('invalid mission counter')
    for name in ('elapsed_seconds','stationary_seconds'):
        x=value.get(name)
        if not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x) or x<0:
            raise ValueError('invalid mission time')
    for name in ('line','crosswalk','calibrated','announcement_ok','finish_announcement_ok'):
        if value.get(name) is not None and not isinstance(value[name],bool): raise ValueError('invalid mission flag')
    if value.get('reason') is not None and not isinstance(value['reason'],str): raise ValueError('invalid mission reason')
    return {key:value.get(key) for key in ('phase','reason','laps','crosswalk_stops','elapsed_seconds',
            'stationary_seconds','line','crosswalk','calibrated','announcement_ok','finish_announcement_ok')}

def parse_model(raw):
    if len(raw)>100_000:raise ValueError('model message too large')
    def reject_constant(value):raise ValueError('non-finite JSON value')
    value=json.loads(raw,parse_constant=reject_constant)
    if not isinstance(value,dict) or value.get('source')!='openvino_race':raise ValueError('invalid model source')
    width,height=value.get('width'),value.get('height')
    if not all(isinstance(v,int) and not isinstance(v,bool) and 0<v<=4096 for v in (width,height)):
        raise ValueError('invalid model image size')
    inference=value.get('inference_ms')
    if not isinstance(inference,(int,float)) or isinstance(inference,bool) or not math.isfinite(inference) or inference<0:
        raise ValueError('invalid inference time')
    detections=value.get('detections')
    if not isinstance(detections,list) or len(detections)>100:raise ValueError('invalid detections')
    for item in detections:
        if not isinstance(item,dict) or item.get('label') not in ('start','crossing'):raise ValueError('invalid detection label')
        score,box=item.get('score'),item.get('box')
        if not isinstance(score,(int,float)) or isinstance(score,bool) or not math.isfinite(score) or not 0<=score<=1:
            raise ValueError('invalid detection score')
        if not isinstance(box,list) or len(box)!=4 or any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) for v in box):
            raise ValueError('invalid detection box')
        if not 0<=box[0]<box[2]<=width or not 0<=box[1]<box[3]<=height:raise ValueError('box outside image')
    return {'source':'openvino_race','width':width,'height':height,'inference_ms':inference,'detections':detections}

def parse_calibration(raw):
    if len(raw)>4096:raise ValueError('calibration message too large')
    value=json.loads(raw)
    if not isinstance(value,dict) or value.get('state') not in ('collecting','complete'):
        raise ValueError('invalid calibration state')
    count,target=value.get('accepted'),value.get('target')
    if not all(isinstance(v,int) and not isinstance(v,bool) for v in (count,target)) or not 0<=count<=target<=60:
        raise ValueError('invalid calibration count')
    if not isinstance(value.get('board_visible'),bool):raise ValueError('invalid calibration flag')
    return {'state':value['state'],'accepted':count,'target':target,'board_visible':value['board_visible']}

def parse_steering(raw):
    if len(raw)>4096:raise ValueError('steering message too large')
    def reject_constant(value):raise ValueError('non-finite JSON value')
    value=json.loads(raw,parse_constant=reject_constant)
    if not isinstance(value,dict) or value.get('source')!='steering_cnn' or not isinstance(value.get('valid'),bool):
        raise ValueError('invalid steering preview')
    if not value['valid']:
        return {'source':'steering_cnn','valid':False}
    steering,inference=value.get('steering'),value.get('inference_ms')
    if any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) for v in (steering,inference)) or inference<0:
        raise ValueError('invalid steering value')
    result={'source':'steering_cnn','valid':True,'steering':steering,'inference_ms':inference}
    for name in ('image_age_ms','processing_ms'):
        if name in value:
            v=value[name]
            if (not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v)
                    or v < (-200 if name=='image_age_ms' else 0)):
                raise ValueError('invalid steering timing')
            result[name]=v
    if 'image_stamp_ns' in value:
        stamp=value['image_stamp_ns']
        if not isinstance(stamp,int) or isinstance(stamp,bool) or not 0 < stamp < 2**63:
            raise ValueError('invalid steering image stamp')
        result['image_stamp_ns']=stamp
    if 'frames_replaced' in value:
        count=value['frames_replaced']
        if not isinstance(count,int) or isinstance(count,bool) or count < 0:
            raise ValueError('invalid replaced frame count')
        result['frames_replaced']=count
    return result

def parse_control(raw):
    if len(raw)>4096:raise ValueError('control diagnostics too large')
    def reject_constant(value):raise ValueError('non-finite JSON value')
    value=json.loads(raw,parse_constant=reject_constant)
    reasons=('waiting','disabled','ready','estop_latched_or_stale','odometry_stale',
             'odometry_limit','stop_not_confirmed','safety_mission_or_command_stop')
    if (not isinstance(value,dict) or value.get('source')!='control_gate'
            or not isinstance(value.get('active'),bool) or value.get('reason') not in reasons
            or not isinstance(value.get('estop_latched'),bool)):
        raise ValueError('invalid control diagnostics')
    fault,age=value.get('feedback_fault'),value.get('feedback_age_s')
    if fault is not None and fault not in ('odometry_limit','stop_not_confirmed'):
        raise ValueError('invalid feedback fault')
    if age is not None and (not isinstance(age,(int,float)) or isinstance(age,bool) or not math.isfinite(age) or age < 0):
        raise ValueError('invalid feedback age')
    return {key:value.get(key) for key in ('source','active','reason','estop_latched','feedback_fault','feedback_age_s')}

def handler_for(store):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/health':
                data=json.dumps(store.status(),ensure_ascii=False,allow_nan=False).encode(); kind='application/json; charset=utf-8'; status=200
            elif path == '/snapshot.jpg':
                data=store.image(); kind='image/jpeg'; status=200 if data else 503
                if data is None: data=b'camera unavailable or stale'
            else: data=b'not found'; kind='text/plain'; status=404
            self.send_response(status); self.send_header('Content-Type',kind); self.send_header('Cache-Control','no-store'); self.send_header('Content-Length',str(len(data))); self.end_headers(); self.wfile.write(data)
    return Handler

def parse_args():
    p=argparse.ArgumentParser(description='ROS 2 只读相机与里程计桥，不输出运动指令')
    p.add_argument('--port',type=int,default=8770)
    p.add_argument('--camera-topic',default='',help='已确认的 ROS 2 相机话题；不设置则不订阅')
    p.add_argument('--camera-type',choices=['compressed','raw'],default='compressed')
    p.add_argument('--odom-topic',default='',help='已确认的 nav_msgs/msg/Odometry 话题；不设置则不订阅')
    p.add_argument('--mission-topic',default='',help='自动任务状态话题；不设置则不订阅')
    p.add_argument('--model-topic',default='',help='原厂模型的只读结果话题')
    p.add_argument('--calibration-topic',default='',help='相机标定状态话题')
    p.add_argument('--steering-topic',default='',help='循迹模型只读预览话题')
    p.add_argument('--control-topic',default='',help='速度仲裁只读诊断话题')
    return p.parse_args()

def main():
    args=parse_args()
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        if args.camera_topic and args.camera_type == 'raw':
            import cv2
            from cv_bridge import CvBridge
    except ImportError as e:
        raise SystemExit(f'ROS 2 依赖未就绪：{e}。请先 source 对应 ROS 2 环境；raw 图像还需要 cv_bridge 与 OpenCV。')
    store=Store()
    class Bridge(Node):
        def __init__(self):
            super().__init__('visual_nav_readonly_bridge')
            self.cv = CvBridge() if args.camera_topic and args.camera_type == 'raw' else None
            if args.camera_topic:
                self.create_subscription(CompressedImage if args.camera_type=='compressed' else Image,args.camera_topic,self.camera,qos_profile_sensor_data)
            if args.odom_topic:
                self.create_subscription(Odometry,args.odom_topic,self.odom,qos_profile_sensor_data)
            if args.mission_topic:
                self.create_subscription(String,args.mission_topic,self.mission,10)
            if args.model_topic:
                self.create_subscription(String,args.model_topic,self.model,10)
            if args.calibration_topic:
                self.create_subscription(String,args.calibration_topic,self.calibration,10)
            if args.steering_topic:
                self.create_subscription(String,args.steering_topic,self.steering,10)
            if args.control_topic:
                self.create_subscription(String,args.control_topic,self.control,10)
            self.create_timer(2.0,self.update_graph)
            self.update_graph()
        def update_graph(self):
            store.set_graph([{'name':name,'types':types} for name,types in self.get_topic_names_and_types()])
        def camera(self,msg):
            try:
                if args.camera_type=='compressed':
                    # Only JPEG is served as image/jpeg; other encodings are ignored.
                    data=bytes(msg.data)
                    if not data.startswith(b'\xff\xd8'): return
                else:
                    image=self.cv.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                    ok,encoded=cv2.imencode('.jpg',image)
                    if not ok: return
                    data=encoded.tobytes()
                store.set_frame(data)
            except Exception as e: self.get_logger().warning(f'相机帧读取失败: {e}')
        def odom(self,msg):
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            values=(p.x,p.y,q.x,q.y,q.z,q.w)
            if not all(math.isfinite(x) for x in values): return
            store.set_pose({'x':p.x,'y':p.y,'yaw_rad':yaw_from_quaternion(q.x,q.y,q.z,q.w),
                            'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id})
        def mission(self,msg):
            try: store.set_mission(parse_mission(msg.data))
            except (ValueError,TypeError): pass
        def model(self,msg):
            try: store.set_model(parse_model(msg.data))
            except (ValueError,TypeError): pass
        def calibration(self,msg):
            try: store.set_calibration(parse_calibration(msg.data))
            except (ValueError,TypeError): pass
        def steering(self,msg):
            try: store.set_steering(parse_steering(msg.data))
            except (ValueError,TypeError): pass
        def control(self,msg):
            try: store.set_control(parse_control(msg.data))
            except (ValueError,TypeError): pass
    rclpy.init()
    node=Bridge()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),handler_for(store))
    worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
    node.get_logger().info(f'只读桥 http://127.0.0.1:{args.port}，camera={args.camera_topic or "未配置"}，odom={args.odom_topic or "未配置"}')
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
