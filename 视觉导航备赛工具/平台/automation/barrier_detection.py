"""Detect candidate blue barriers inside observed yellow lane boundaries.

Image-space geometry is provisional and must be calibrated on the actual track.
This module does not publish motion or an independent safety-clear signal.
"""
import math
import numpy as np

from automation.steering_candidate import finite


def validate_config(config):
    if not isinstance(config, dict) or not isinstance(config.get('calibrated'), bool):
        raise ValueError('barrier calibrated must be true or false')
    roi = config.get('roi')
    if (not isinstance(roi, list) or len(roi) != 4 or not all(finite(v) for v in roi)
            or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1)):
        raise ValueError('barrier ROI must be [x1,y1,x2,y2] in 0..1')
    for name in ('yellow', 'blue'):
        value = config.get(name)
        if not isinstance(value, dict):
            raise ValueError(name + ' HSV bounds missing')
        low, high = value.get('lower'), value.get('upper')
        if (not isinstance(low, list) or not isinstance(high, list) or len(low) != 3 or len(high) != 3
                or any(not isinstance(v, int) or isinstance(v, bool) for v in low + high)
                or any(not 0 <= low[i] <= high[i] <= (179 if i == 0 else 255) for i in range(3))):
            raise ValueError(name + ' HSV bounds invalid')
    for name in ('lane_center_x', 'min_lane_width', 'min_valid_row_fraction',
                 'min_component_area', 'min_component_height'):
        value = config.get(name)
        if not finite(value) or not 0 < value <= 1:
            raise ValueError(name + ' must be in (0,1]')
    margin = config.get('boundary_margin')
    if not finite(margin) or not 0 <= margin < .25:
        raise ValueError('boundary margin invalid')
    max_components = config.get('max_components')
    if not isinstance(max_components, int) or isinstance(max_components, bool) or not 1 <= max_components <= 100:
        raise ValueError('max components invalid')
    return config


def detect_barriers(image, config):
    """Return normalized boxes and confidence in the lane observation."""
    import cv2

    validate_config(config)
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or not image.size:
        raise ValueError('expected a non-empty RGB uint8 image')
    height, width = image.shape[:2]
    if width < 32 or height < 32 or width * height > 16_000_000:
        raise ValueError('image dimensions invalid')
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    masks = {}
    for name in ('yellow', 'blue'):
        bounds = config[name]
        masks[name] = cv2.inRange(hsv, np.array(bounds['lower'], dtype=np.uint8),
                                 np.array(bounds['upper'], dtype=np.uint8)) > 0
    x1, y1, x2, y2 = config['roi']
    left, top = int(x1 * width), int(y1 * height)
    right, bottom = min(width, math.ceil(x2 * width)), min(height, math.ceil(y2 * height))
    center = int(config['lane_center_x'] * width)
    if right <= left or bottom <= top or not left < center < right:
        raise ValueError('lane center lies outside ROI')
    mask = np.zeros((height, width), dtype=np.uint8)
    observed = []
    margin = int(config['boundary_margin'] * width)
    for y in range(top, bottom):
        row = masks['yellow'][y, left:right]
        edges = np.diff(np.r_[False, row, False].astype(np.int8))
        starts, ends = np.flatnonzero(edges == 1) + left, np.flatnonzero(edges == -1) + left
        runs = [(int(a), int(b)) for a, b in zip(starts, ends) if b - a >= 2]
        left_edges = [b for a, b in runs if b <= center]
        right_edges = [a for a, b in runs if a >= center]
        if not left_edges or not right_edges:
            continue
        lo, hi = max(left_edges) + margin, min(right_edges) - margin
        if hi - lo < config['min_lane_width'] * width:
            continue
        observed.append((lo, hi))
        mask[y, lo:hi] = masks['blue'][y, lo:hi].astype(np.uint8)
    fraction = len(observed) / (bottom - top)
    if not observed:
        return {'valid': False, 'calibrated': config['calibrated'], 'lane': None,
                'valid_row_fraction': 0.0, 'boxes': []}
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    boxes = []
    for x, y, w, h, area in stats[1:]:
        if area < config['min_component_area'] * width * height or h < config['min_component_height'] * height:
            continue
        boxes.append({'box': [round(x / width, 4), round(y / height, 4),
                              round((x + w) / width, 4), round((y + h) / height, 4)],
                      'area_fraction': round(float(area / (width * height)), 5)})
    boxes.sort(key=lambda item: item['box'][3], reverse=True)
    lane = [round(float(np.median([x[0] for x in observed])) / width, 4),
            round(float(np.median([x[1] for x in observed])) / width, 4)]
    valid = bool(config['calibrated'] and fraction >= config['min_valid_row_fraction']
                 and len(boxes) <= config['max_components'])
    return {'valid': valid, 'calibrated': config['calibrated'], 'lane': lane,
            'valid_row_fraction': round(fraction, 4), 'boxes': boxes[:config['max_components']]}
