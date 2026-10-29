"""Validate timestamped vendor-model observations for crosswalk decisions."""
import math
import re

from automation.steering_candidate import finite, fresh_frame
from automation.detector import region, validate_config as validate_stripes


def geometry_settings(config):
    value=config.get('stripe_geometry', {'white_threshold':.75,'min_coverage':.35,
               'min_band_fraction':.02,'min_bands':3,'max_bands':15})
    if not isinstance(value,dict):raise ValueError('stripe geometry must be an object')
    settings=dict(value,roi=[0.,0.,1.,1.],axis='x')
    validate_stripes({'calibrated':False,'start_line':settings,'crosswalk':settings})
    return settings


def verify_crosswalk_box(image, box, config):
    height,width=image.shape[:2]
    if (not isinstance(box,list) or len(box)!=4 or not all(finite(x) for x in box)
            or not (0<=box[0]<box[2]<=width and 0<=box[1]<box[3]<=height)):
        raise ValueError('crosswalk box outside image')
    settings=geometry_settings(config)
    settings['roi']=[box[0]/width,box[1]/height,box[2]/width,box[3]/height]
    value=region(image,settings)
    return {'stripe_verified':value['detected'],'stripe_bands':value['bands']}


def validate_model_config(config):
    if not isinstance(config, dict) or not isinstance(config.get('calibrated'), bool):
        raise ValueError('model calibrated must be true or false')
    score = config.get('min_confidence')
    roi = config.get('roi')
    if not finite(score) or not 0 < score <= 1:
        raise ValueError('model confidence must be in (0,1]')
    if (not isinstance(roi, list) or len(roi) != 4 or not all(finite(x) for x in roi)
            or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1)):
        raise ValueError('model ROI must be [x1,y1,x2,y2] in 0..1')
    if not all(finite(config.get(k)) and config[k] > 0 for k in ('max_source_age_s', 'max_gap_s')):
        raise ValueError('model timeouts must be positive')
    if 'class_order' in config and (not isinstance(config['class_order'],list)
            or len(config['class_order'])!=2 or not all(isinstance(x,str) for x in config['class_order'])
            or set(config['class_order'])!={'start','crossing'}):
        raise ValueError('class order must contain start and crossing once each')
    if 'model_sha256' in config and (not isinstance(config['model_sha256'],str)
            or not re.fullmatch('[0-9a-f]{64}',config['model_sha256'])):
        raise ValueError('model SHA-256 invalid')
    if not isinstance(config.get('require_stripes',False),bool):
        raise ValueError('require_stripes must be boolean')
    if config.get('require_stripes'):
        geometry_settings(config)
    return config


class ModelCrosswalk:
    """Each accepted result contributes one observation; repeats do not refresh it."""
    def __init__(self, config):
        self.config = validate_model_config(config)
        self.latest_stamp_ns = None
        self.stamp_ns = None
        self.received_at = None
        self.detected = None

    def invalidate(self):
        self.stamp_ns, self.received_at, self.detected = None, None, None

    def update(self, value, now, ros_now_ns):
        if not finite(now) or not isinstance(value, dict) or value.get('source') != 'openvino_race' or value.get('valid') is not True:
            self.invalidate()
            return None
        stamp, width, height = value.get('stamp_ns'), value.get('width'), value.get('height')
        if not fresh_frame(stamp, ros_now_ns, self.config['max_source_age_s']):
            self.invalidate()
            return None
        if (('class_order' in self.config and value.get('class_order')!=self.config['class_order'])
                or ('model_sha256' in self.config and value.get('model_sha256')!=self.config['model_sha256'])
                or (self.config.get('require_stripes') and value.get('stripe_geometry')!=geometry_settings(self.config))):
            self.invalidate()
            return None
        # Out-of-order and repeated results cannot count as additional frames.
        if self.latest_stamp_ns is not None and stamp <= self.latest_stamp_ns:
            return None
        if (not all(isinstance(v, int) and not isinstance(v, bool) and 0 < v <= 4096 for v in (width, height))
                or not finite(value.get('inference_ms')) or value['inference_ms'] < 0
                or not isinstance(value.get('detections'), list) or len(value['detections']) > 100):
            self.invalidate()
            return None
        roi = self.config['roi']
        detected = False
        for item in value['detections']:
            if not isinstance(item, dict) or item.get('label') not in ('crossing', 'start'):
                self.invalidate()
                return None
            if 'class_order' in self.config:
                class_id=item.get('class_id')
                if (not isinstance(class_id,int) or isinstance(class_id,bool) or not 0<=class_id<2
                        or self.config['class_order'][class_id]!=item['label']):
                    self.invalidate()
                    return None
            score, box = item.get('score'), item.get('box')
            if (not finite(score) or not 0 <= score <= 1 or not isinstance(box, list)
                    or len(box) != 4 or not all(finite(v) for v in box)
                    or not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height)):
                self.invalidate()
                return None
            cx, bottom = (box[0] + box[2]) / (2 * width), box[3] / height
            if item['label']=='crossing' and self.config.get('require_stripes'):
                bands=item.get('stripe_bands')
                if (not isinstance(item.get('stripe_verified'),bool) or not isinstance(bands,int)
                        or isinstance(bands,bool) or bands<0):
                    self.invalidate()
                    return None
                if not item['stripe_verified']:
                    continue
                shape=geometry_settings(self.config)
                if not shape['min_bands']<=bands<=shape['max_bands']:
                    self.invalidate()
                    return None
            if (item['label'] == 'crossing' and score >= self.config['min_confidence']
                    and roi[0] <= cx <= roi[2] and roi[1] <= bottom <= roi[3]):
                detected = True
        self.latest_stamp_ns, self.stamp_ns = stamp, stamp
        self.received_at, self.detected = float(now), detected
        return detected

    def available(self, now, ros_now_ns):
        return (finite(now) and self.received_at is not None
                and 0 <= now - self.received_at <= self.config['max_gap_s']
                and fresh_frame(self.stamp_ns, ros_now_ns, self.config['max_source_age_s']))
