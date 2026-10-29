#!/usr/bin/env python3
"""Review captured checkerboards and write a separate ROS camera calibration."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from ros2_camera_calibrate import signature


def main():
    parser = argparse.ArgumentParser(description='复核棋盘格采样并生成独立的相机内参')
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--out', required=True, type=Path, help='新建的结果 YAML')
    parser.add_argument('--max-original-view-rms', type=float, default=1.5)
    args = parser.parse_args()
    if not math.isfinite(args.max_original_view_rms) or args.max_original_view_rms <= 0:
        parser.error('每张画面的误差上限必须大于零')

    root = args.capture_dir.expanduser()
    with (root / 'camera-intrinsics.yaml').open(encoding='utf-8') as f:
        original = yaml.safe_load(f)
    cols, rows = original['board']['inner_corners']
    square = float(original['board']['square_size_m'])
    width, height = original['image_width'], original['image_height']
    images = sorted(root.glob('sample-*.jpg'))
    original_errors = original['per_view_rms_px']
    if len(images) != original['sample_count'] or len(images) != len(original_errors):
        raise SystemExit('采样图片和原始标定记录数量不一致')
    if not (3 <= cols <= 30 and 3 <= rows <= 30 and square > 0):
        raise SystemExit('原始棋盘格参数无效')

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    points = []
    names = []
    signatures = []
    excluded = []
    for path, error in zip(images, original_errors):
        if not math.isfinite(float(error)):
            raise SystemExit('原始标定记录包含无效误差')
        if error > args.max_original_view_rms:
            excluded.append({'file': path.name, 'original_rms_px': float(error)})
            continue
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (height, width):
            raise SystemExit('无法读取尺寸一致的采样图片：' + path.name)
        ok, corners = cv2.findChessboardCorners(image, (cols, rows), flags)
        if not ok:
            raise SystemExit('无法重新识别角点：' + path.name)
        grid = corners.reshape(rows, cols, 2)
        horizontal = np.linalg.norm(np.diff(grid, axis=1), axis=2).flatten()
        vertical = np.linalg.norm(np.diff(grid, axis=0), axis=2).flatten()
        spacing = float(np.median(np.concatenate((horizontal, vertical))))
        window = max(2, min(7, int(spacing * .22)))
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001)
        cv2.cornerSubPix(image, corners, (window, window), (-1, -1), criteria)
        points.append(corners)
        names.append(path.name)
        xy = [(float(x), float(y)) for x, y in corners.reshape(-1, 2)]
        signatures.append(signature(xy, cols, rows, width, height))
    if len(points) < 12:
        raise SystemExit('高质量画面不足 12 张，请重新采集')

    board = np.zeros((cols * rows, 3), np.float32)
    board[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    # The simpler four-coefficient lens model avoids an unconstrained k3 term
    # dominating rectification near the edge of this small training set.
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [board.copy() for _ in points], points, (width, height), None, None,
        flags=cv2.CALIB_FIX_K3)
    if not math.isfinite(float(rms)) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise SystemExit('标定结果无效，请重新采集')
    optimal, _ = cv2.getOptimalNewCameraMatrix(matrix, distortion, (width, height), 1, (width, height))
    projection = np.zeros((3, 4), dtype=float)
    projection[:3, :3] = optimal
    per_view = []
    for corners, rvec, tvec in zip(points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(board, rvec, tvec, matrix, distortion)
        per_view.append(float(np.sqrt(np.mean(np.sum((corners - projected) ** 2, axis=2)))))
    report = {
        'image_width': width, 'image_height': height, 'camera_name': original['camera_name'],
        'distortion_model': 'plumb_bob',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': matrix.flatten().tolist()},
        'distortion_coefficients': {'rows': 1, 'cols': int(distortion.size), 'data': distortion.flatten().tolist()},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).flatten().tolist()},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': projection.flatten().tolist()},
        'reprojection_rms_px': float(rms), 'sample_count': len(points),
        'per_view_rms_px': per_view, 'sample_files': names,
        'board': {'inner_corners': [cols, rows], 'square_size_m': square},
        'center_span': [max(s[i] for s in signatures) - min(s[i] for s in signatures) for i in (0, 1)],
        'initial_sample_count': len(images), 'excluded_samples': excluded,
        'calibration_flags': ['fix_k3'],
    }
    with args.out.expanduser().open('x', encoding='utf-8') as f:
        yaml.safe_dump(report, f, sort_keys=False, allow_unicode=True)
    print(json.dumps({'output': str(args.out), 'rms_px': round(float(rms), 3),
                      'sample_count': len(points), 'excluded': excluded,
                      'center_span': report['center_span']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
