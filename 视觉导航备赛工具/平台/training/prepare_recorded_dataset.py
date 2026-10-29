#!/usr/bin/env python3
"""Prepare ROS 2 manual-driving recordings for steering-model training.

The steering target is the recorded angular velocity command (rad/s), not a
measured wheel angle. Source files are never changed or overwritten.
"""
import argparse
import csv
import json
import math
import shutil
import tempfile
from pathlib import Path

from PIL import Image, UnidentifiedImageError

from training.prepare_vendor_dataset import sha256


def prepare(source, output, validation_fraction=0.2, test_fraction=0.2,
            gap_seconds=3.0, min_forward_speed=0.02):
    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().absolute()
    if not source.is_dir() or not (source / 'data.csv').is_file():
        raise ValueError('采集目录须包含 data.csv 和 images')
    if output.exists():
        raise ValueError('输出目录已存在，请使用新目录')
    if (not 0.05 <= validation_fraction <= 0.5 or not 0 <= test_fraction <= 0.5
            or validation_fraction + test_fraction > 0.8
            or not math.isfinite(gap_seconds) or gap_seconds < 0
            or not math.isfinite(min_forward_speed) or min_forward_speed <= 0):
        raise ValueError('数据集划分或速度参数无效')
    rows, sizes, seen = [], set(), set()
    skipped = 0
    previous_stamp = 0
    with (source / 'data.csv').open(encoding='utf-8-sig', newline='') as handle:
        reader = csv.DictReader(handle)
        needed = {'image', 'linear_x', 'angular_z', 'image_stamp_ns', 'command_age_ms'}
        if not reader.fieldnames or not needed.issubset(reader.fieldnames):
            raise ValueError('不是支持的 ROS 2 采集数据格式')
        for line, row in enumerate(reader, 2):
            try:
                stamp = int(row['image_stamp_ns'])
                forward, steering, age = (float(row[k]) for k in ('linear_x', 'angular_z', 'command_age_ms'))
                if stamp <= previous_stamp or not all(math.isfinite(v) for v in (forward, steering, age)) or not 0 <= age <= 2000:
                    raise ValueError('时间戳或控制量无效')
                previous_stamp = stamp
                relative = Path(row['image'])
                image = (source / relative).resolve()
                if relative.is_absolute() or source not in image.parents or not image.is_file() or image in seen:
                    raise ValueError('图片路径无效或重复')
                seen.add(image)
                with Image.open(image) as picture:
                    size = tuple(picture.size)
                    picture.verify()
            except (ValueError, TypeError, OSError, UnidentifiedImageError) as error:
                raise ValueError(f'第 {line} 行图片、时间戳或标签无效') from error
            # Stop/reverse examples are not target data for a forward-driving model.
            if forward < min_forward_speed:
                skipped += 1
                continue
            sizes.add(size)
            rows.append({'image': image, 'timestamp_ns': stamp, 'steering': steering})
    if len(rows) < 20 or len(sizes) != 1:
        raise ValueError('前进样本过少或图片尺寸不一致')
    val_start = int(len(rows) * (1 - validation_fraction - test_fraction))
    test_start = int(len(rows) * (1 - test_fraction)) if test_fraction else len(rows)
    test = rows[test_start:]
    validation = rows[val_start:test_start]
    gap_ns = int(gap_seconds * 1_000_000_000)
    if test:
        validation = [r for r in validation if r['timestamp_ns'] < test[0]['timestamp_ns'] - gap_ns]
    training = [r for r in rows[:val_start] if r['timestamp_ns'] < rows[val_start]['timestamp_ns'] - gap_ns]
    gap_count = len(rows) - len(training) - len(validation) - len(test)
    if len(training) < 10 or len(validation) < 5 or (test_fraction and len(test) < 5):
        raise ValueError('切分后样本不足，请采集更长的前进行驶数据')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.recorded-stage-', dir=str(output.parent)) as scratch:
        stage = Path(scratch) / 'dataset'
        images = stage / 'images'
        images.mkdir(parents=True)
        for index, row in enumerate(rows):
            # Unique generated names also avoid trusting source path names.
            row['portable_name'] = f'{index:07d}{row["image"].suffix.lower()}'
            shutil.copy2(row['image'], images / row['portable_name'])
        groups = [('train.csv', training), ('validation.csv', validation)]
        if test:
            groups.append(('test.csv', test))
        for name, group in groups:
            with (stage / name).open('w', encoding='utf-8', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['image_path', 'steering'])
                writer.writerows([('images/' + row['portable_name'], row['steering']) for row in group])
        values = [r['steering'] for r in rows]
        metadata = {
            'source': 'ROS 2 manual-driving recording',
            'source_csv_sha256': sha256(source / 'data.csv'),
            'samples': len(rows), 'skipped_stop_or_reverse': skipped,
            'training_samples': len(training), 'validation_samples': len(validation),
            'test_samples': len(test), 'gap_samples': gap_count,
            'image_size_wh': list(next(iter(sizes))),
            'split': 'chronological', 'validation_fraction': validation_fraction,
            'test_fraction': test_fraction, 'gap_seconds': gap_seconds,
            'min_forward_speed': min_forward_speed,
            'steering_min': min(values), 'steering_max': max(values),
            'label': 'angular_z command in rad/s; not measured vehicle speed or wheel angle',
            'note': 'The holdout is from the same recording, not an independent field test.'
        }
        (stage / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        if output.exists():
            raise ValueError('输出目录已存在，请使用新目录')
        stage.rename(output)
    return metadata


def main():
    parser = argparse.ArgumentParser(description='整理 ROS 2 手动驾驶采集数据')
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--validation-fraction', type=float, default=0.2)
    parser.add_argument('--test-fraction', type=float, default=0.2)
    parser.add_argument('--gap-seconds', type=float, default=3.0)
    parser.add_argument('--min-forward-speed', type=float, default=0.02)
    args = parser.parse_args()
    try:
        print(json.dumps(prepare(args.source, args.output, args.validation_fraction,
                                 args.test_fraction, args.gap_seconds, args.min_forward_speed), ensure_ascii=False, indent=2))
    except (ValueError, OSError, csv.Error) as error:
        parser.exit(2, '整理失败：' + str(error) + '\n')


if __name__ == '__main__':
    main()
