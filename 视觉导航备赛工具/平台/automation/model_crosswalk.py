"""Validate timestamped vendor-model observations for crosswalk decisions."""
import math

from automation.steering_candidate import finite, fresh_frame


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
            score, box = item.get('score'), item.get('box')
            if (not finite(score) or not 0 <= score <= 1 or not isinstance(box, list)
                    or len(box) != 4 or not all(finite(v) for v in box)
                    or not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height)):
                self.invalidate()
                return None
            cx, bottom = (box[0] + box[2]) / (2 * width), box[3] / height
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
