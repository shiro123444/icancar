#!/usr/bin/env python3
"""Prepare a portable copy of the vendor's recorded driving data.

The original CSV contains vehicle-specific absolute paths. This script keeps
the source untouched, validates the image/label pairs and splits one recorded
sequence chronologically, with a time gap between training and validation.
"""
import argparse
import csv
import hashlib
import json
import math
import shutil
import tempfile
from pathlib import Path

from PIL import Image, UnidentifiedImageError


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def prepare(source, output, validation_fraction=0.2, gap_seconds=3.0, test_fraction=0.0):
    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().absolute()
    if not source.is_dir() or not (source / 'data.csv').is_file():
        raise ValueError('源目录必须包含厂家 data.csv')
    if output.exists():
        raise ValueError('输出目录已存在，请使用新的目录')
    if (not 0.05 <= validation_fraction <= 0.5 or not 0 <= test_fraction <= 0.5
            or validation_fraction + test_fraction > .8 or not math.isfinite(gap_seconds) or gap_seconds < 0):
        raise ValueError('验证比例或间隔无效')
    rows = []
    seen = set()
    sizes = set()
    with (source / 'data.csv').open(encoding='utf-8-sig', newline='') as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or not {'image_path', 'steering'}.issubset(reader.fieldnames):
            raise ValueError('CSV 缺少 image_path 或 steering 字段')
        for line, row in enumerate(reader, 2):
            # Only use the basename: absolute vehicle paths do not exist on Mac.
            # Input images must be regular files directly under the source folder.
            name = Path(row['image_path']).name
            image = source / name
            if not name or name in seen or image.is_symlink() or not image.is_file():
                raise ValueError(f'第 {line} 行图片缺失或重复: {name}')
            try:
                timestamp = int(image.stem)
                steering = float(row['steering'])
                if not math.isfinite(steering):
                    raise ValueError('non-finite label')
                with Image.open(image) as picture:
                    sizes.add(tuple(picture.size))
                    picture.verify()
            except (ValueError, TypeError, OSError, UnidentifiedImageError) as error:
                raise ValueError(f'第 {line} 行图片或标签无效: {name}') from error
            rows.append({'filename': name, 'timestamp_ms': timestamp, 'steering': steering})
            seen.add(name)
    if len(rows) < 20 or len(sizes) != 1:
        raise ValueError('样本过少或图片尺寸不一致')
    rows.sort(key=lambda r: r['timestamp_ms'])
    if any(a['timestamp_ms'] >= b['timestamp_ms'] for a, b in zip(rows, rows[1:])):
        raise ValueError('照片时间戳重复')
    val_start = int(len(rows) * (1 - validation_fraction - test_fraction))
    test_start = int(len(rows) * (1 - test_fraction)) if test_fraction else len(rows)
    test = rows[test_start:]
    validation = rows[val_start:test_start]
    if test:
        validation = [row for row in validation if row['timestamp_ms'] < test[0]['timestamp_ms'] - gap_seconds * 1000]
    boundary = rows[val_start]['timestamp_ms']
    training = [row for row in rows[:val_start] if row['timestamp_ms'] < boundary - gap_seconds * 1000]
    gap_count = len(rows) - len(training) - len(validation) - len(test)
    if len(training) < 10 or len(validation) < 5 or (test_fraction and len(test) < 5):
        raise ValueError('切分后训练集、验证集或测试集样本过少')

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.dataset-stage-', dir=str(output.parent)) as scratch:
        stage = Path(scratch) / 'dataset'
        images = stage / 'images'
        images.mkdir(parents=True)
        # Keep the gap samples for later regrouping, but not in either manifest.
        for row in rows:
            shutil.copy2(source / row['filename'], images / row['filename'])
        groups = [('train.csv', training), ('validation.csv', validation)]
        if test:
            groups.append(('test.csv', test))
        for name, group in groups:
            with (stage / name).open('w', encoding='utf-8', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['image_path', 'steering'])
                writer.writerows([('images/' + row['filename'], row['steering']) for row in group])
        metadata = {
            'source': '2025 DEEPBOT vendor driving dataset',
            'source_csv_sha256': sha256(source / 'data.csv'),
            'samples': len(rows), 'training_samples': len(training),
            'validation_samples': len(validation), 'test_samples': len(test), 'gap_samples': gap_count,
            'image_size_wh': list(next(iter(sizes))),
            'first_timestamp_ms': rows[0]['timestamp_ms'], 'last_timestamp_ms': rows[-1]['timestamp_ms'],
            'split': 'chronological', 'validation_fraction': validation_fraction, 'test_fraction': test_fraction, 'gap_seconds': gap_seconds,
            'label': 'steering; the currently installed vendor collector uses /odom twist.angular.z, but this historical capture version is unverified',
            'note': 'Same venue/recording sequence; validation is not an independent field test. Historical 224x224 images differ from the current collector output size.'
        }
        (stage / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        if output.exists():
            raise ValueError('输出目录已存在，请使用新的目录')
        stage.rename(output)
    return metadata


def main():
    parser = argparse.ArgumentParser(description='整理厂家历史驾驶样本，不修改原始数据')
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--validation-fraction', type=float, default=0.2)
    parser.add_argument('--test-fraction', type=float, default=0.0)
    parser.add_argument('--gap-seconds', type=float, default=3.0)
    args = parser.parse_args()
    try:
        print(json.dumps(prepare(args.source, args.output, args.validation_fraction, args.gap_seconds, args.test_fraction), ensure_ascii=False, indent=2))
    except (ValueError, OSError, csv.Error) as error:
        parser.exit(2, '整理失败：' + str(error) + '\n')


if __name__ == '__main__':
    main()
