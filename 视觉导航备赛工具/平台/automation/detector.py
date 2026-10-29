"""Configurable image-region stripe detector; calibrate with the actual camera."""
import math
import numpy as np


def validate_config(config):
    if not isinstance(config, dict) or not isinstance(config.get('calibrated'), bool):
        raise ValueError('calibrated must be true or false')
    for name in ('start_line', 'crosswalk'):
        entry = config.get(name)
        if not isinstance(entry, dict):
            raise ValueError(name + ' configuration missing')
        roi = entry.get('roi')
        if (not isinstance(roi, list) or len(roi) != 4 or
                not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in roi) or
                not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1)):
            raise ValueError(name + ' roi must be [x1,y1,x2,y2] in 0..1')
        if entry.get('axis') not in ('x', 'y'):
            raise ValueError(name + ' axis must be x or y')
        for key in ('white_threshold', 'min_coverage', 'min_band_fraction'):
            x = entry.get(key)
            if not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x) or not 0 < x <= 1:
                raise ValueError(name + ' ' + key + ' must be in (0,1]')
        low, high = entry.get('min_bands'), entry.get('max_bands')
        if not all(isinstance(x, int) and not isinstance(x, bool) for x in (low, high)) or not 1 <= low <= high:
            raise ValueError(name + ' band range invalid')
    return config


def region(image, settings):
    height, width = image.shape[:2]
    x1, y1, x2, y2 = settings['roi']
    left, top = int(x1 * width), int(y1 * height)
    right, bottom = min(width, math.ceil(x2 * width)), min(height, math.ceil(y2 * height))
    if right <= left or bottom <= top:
        raise ValueError('image too small for ROI')
    cropped = image[top:bottom, left:right]
    # RGB luminance approximation. Avoid overflow of uint8 arithmetic.
    gray = np.mean(cropped.astype(np.float32), axis=2)
    white = gray >= settings['white_threshold'] * 255
    projection = white.mean(axis=0 if settings['axis'] == 'x' else 1)
    active = projection >= settings['min_coverage']
    edges = np.diff(np.r_[False, active, False].astype(np.int8))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    min_width = max(1, math.ceil(len(projection) * settings['min_band_fraction']))
    bands = [(int(a), int(b)) for a, b in zip(starts, ends) if b - a >= min_width]
    return {'detected': settings['min_bands'] <= len(bands) <= settings['max_bands'],
            'bands': len(bands), 'white_fraction': round(float(white.mean()), 4),
            'rect': [left, top, right, bottom]}


def detect(image, config):
    validate_config(config)
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or not image.size:
        raise ValueError('expected a non-empty RGB uint8 image')
    return {name: region(image, config[name]) for name in ('start_line', 'crosswalk')}
