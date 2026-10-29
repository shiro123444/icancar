#!/usr/bin/env python3
"""Inspect saved photos or calculate a camera calibration without vehicle access."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from ros2_camera_calibrate import signature


def find_board(path, cols, rows):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError('无法读取图片：' + path.name)
    height, width = image.shape
    if width > 8192 or height > 8192:
        raise ValueError('图片尺寸过大：' + path.name)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(image, (cols, rows), flags)
    if not ok:
        return (width, height), None
    grid = corners.reshape(rows, cols, 2)
    horizontal = np.linalg.norm(np.diff(grid, axis=1), axis=2).flatten()
    vertical = np.linalg.norm(np.diff(grid, axis=0), axis=2).flatten()
    spacing = float(np.median(np.concatenate((horizontal, vertical))))
    if spacing < 8:
        return (width, height), None
    window = max(2, min(7, int(spacing * .22)))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001)
    cv2.cornerSubPix(image, corners, (window, window), (-1, -1), criteria)
    return (width, height), corners


def view_error(board, points, rvec, tvec, matrix, distortion):
    projected, _ = cv2.projectPoints(board, rvec, tvec, matrix, distortion)
    return float(np.sqrt(np.mean(np.sum((points - projected) ** 2, axis=2))))


def fit(board, points, image_size):
    result = cv2.calibrateCamera([board.copy() for _ in points], points, image_size,
                                 None, None, flags=cv2.CALIB_FIX_K3)
    rms, matrix, distortion, rvecs, tvecs = result
    errors = [view_error(board, p, r, t, matrix, distortion)
              for p, r, t in zip(points, rvecs, tvecs)]
    return float(rms), matrix, distortion, errors


def calculate(root, output):
    with (root / 'session.json').open(encoding='utf-8') as f:
        session = json.load(f)
    cols = session['board']['cols']
    rows = session['board']['rows']
    square = session['board']['square_mm'] / 1000
    board = np.zeros((cols * rows, 3), np.float32)
    board[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    names, points, skipped = [], [], []
    image_size = None
    for sample in session['photos']:
        if not sample['enabled']:
            continue
        path = root / sample['file']
        size, corners = find_board(path, cols, rows)
        if image_size is not None and size != image_size:
            raise ValueError('照片分辨率不同，请只使用同一相机和分辨率')
        image_size = size
        if corners is None:
            skipped.append({'file': sample['file'], 'reason': '未识别到完整棋盘格'})
            continue
        names.append(sample['file'])
        points.append(corners)
    if len(points) < 12:
        raise ValueError('至少需要 12 张识别到棋盘格的照片，目前有 %d 张' % len(points))
    rms, matrix, distortion, errors = fit(board, points, image_size)
    # A badly blurred moving board can be detected yet still spoil a fit.
    # Exclude only conspicuous residuals, and retain at least 12 usable views.
    threshold = max(1.5, float(np.median(errors)) * 3)
    keep = [i for i, error in enumerate(errors) if error <= threshold]
    if len(keep) >= 12 and len(keep) < len(points):
        for i, error in enumerate(errors):
            if i not in keep:
                skipped.append({'file': names[i], 'reason': '画面偏差较大', 'initial_rms_px': error})
        names = [names[i] for i in keep]
        points = [points[i] for i in keep]
        rms, matrix, distortion, errors = fit(board, points, image_size)
    if not math.isfinite(rms) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise ValueError('标定结果无效，请增加不同角度的清晰照片')
    width, height = image_size
    optimal, _ = cv2.getOptimalNewCameraMatrix(matrix, distortion, image_size, 1, image_size)
    projection = np.zeros((3, 4), dtype=float)
    projection[:3, :3] = optimal
    signatures = []
    for corners in points:
        xy = [(float(x), float(y)) for x, y in corners.reshape(-1, 2)]
        signatures.append(signature(xy, cols, rows, width, height))
    span = [max(s[i] for s in signatures) - min(s[i] for s in signatures) for i in (0, 1)]
    issues = []
    if len(points) < 15:
        issues.append('建议补充至至少 15 张不同角度的照片')
    if min(span) < .2:
        issues.append('棋盘格在画面中的位置覆盖较小')
    if rms > 1:
        issues.append('重投影误差较大，建议补充清晰且贴平的照片')
    if not (.2 * width < matrix[0, 0] < 5 * width and .2 * height < matrix[1, 1] < 5 * height
            and 0 < matrix[0, 2] < width and 0 < matrix[1, 2] < height):
        issues.append('相机参数超出常见范围，请重新采集')
    report = {
        'image_width': width, 'image_height': height, 'camera_name': 'visual_race_camera',
        'distortion_model': 'plumb_bob',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': matrix.flatten().tolist()},
        'distortion_coefficients': {'rows': 1, 'cols': int(distortion.size), 'data': distortion.flatten().tolist()},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).flatten().tolist()},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': projection.flatten().tolist()},
        'reprojection_rms_px': rms, 'sample_count': len(points), 'per_view_rms_px': errors,
        'sample_files': names, 'excluded_samples': skipped,
        'board': {'inner_corners': [cols, rows], 'square_size_m': square},
        'center_span': span, 'calibration_flags': ['fix_k3'],
        'quality': 'review' if issues else 'ok', 'quality_notes': issues,
        'session_id': session['id'], 'input_revision': session['revision'],
    }
    with output.open('x', encoding='utf-8') as f:
        yaml.safe_dump(report, f, sort_keys=False, allow_unicode=True)
    return {'ok': True, 'file': output.name, 'rms_px': round(rms, 3), 'sample_count': len(points),
            'skipped': skipped, 'center_span': span, 'quality': report['quality'], 'quality_notes': issues,
            'input_revision': session['revision']}


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='action', required=True)
    inspect = sub.add_parser('inspect')
    inspect.add_argument('image', type=Path)
    inspect.add_argument('--cols', type=int, required=True)
    inspect.add_argument('--rows', type=int, required=True)
    compute = sub.add_parser('compute')
    compute.add_argument('session', type=Path)
    compute.add_argument('output', type=Path)
    args = parser.parse_args()
    try:
        if args.action == 'inspect':
            size, corners = find_board(args.image, args.cols, args.rows)
            result = {'ok': True, 'detected': corners is not None, 'width': size[0], 'height': size[1]}
        else:
            result = calculate(args.session, args.output)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    except (ValueError, OSError, KeyError, cv2.error) as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=False))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
