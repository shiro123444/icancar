#!/usr/bin/env python3
"""Manage saved camera calibration sessions without the web interface."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from calibration import CalibrationSessions


def main():
    parser = argparse.ArgumentParser(description='相机标定照片与结果管理')
    parser.add_argument('--python', default=str(ROOT / '.venv-calibration' / 'bin' / 'python'))
    commands = parser.add_subparsers(dest='action', required=True)
    create = commands.add_parser('create', help='新建采集组')
    create.add_argument('--cols', type=int, default=8)
    create.add_argument('--rows', type=int, default=6)
    create.add_argument('--square-mm', type=float, default=20)
    commands.add_parser('list', help='列出采集组')
    show = commands.add_parser('show', help='查看采集组')
    show.add_argument('id')
    capture = commands.add_parser('capture', help='保存一张小车相机画面')
    capture.add_argument('id')
    capture.add_argument('--telemetry-port', type=int, default=8770)
    add = commands.add_parser('import', help='导入本机相机照片')
    add.add_argument('id')
    add.add_argument('photos', nargs='+', type=Path)
    for action in ('exclude', 'include'):
        toggle = commands.add_parser(action, help='排除或恢复某张照片')
        toggle.add_argument('id')
        toggle.add_argument('photo')
    compute = commands.add_parser('compute', help='从保存的照片计算标定参数')
    compute.add_argument('id')
    export = commands.add_parser('export', help='导出照片和计算结果')
    export.add_argument('id')
    export.add_argument('output', type=Path)
    args = parser.parse_args()
    manager = CalibrationSessions(ROOT / 'data' / '相机标定', args.python,
                                  ROOT / 'tools' / 'calibration_worker.py')
    try:
        if args.action == 'create':
            result = manager.create(args.cols, args.rows, args.square_mm)
        elif args.action == 'list':
            result = manager.list()
        elif args.action == 'show':
            result = manager.detail(args.id)
        elif args.action == 'capture':
            result = manager.capture(args.id, args.telemetry_port)
        elif args.action == 'import':
            for path in args.photos:
                result = manager.add_photo(args.id, path.expanduser().read_bytes(), source='import')
        elif args.action in ('exclude', 'include'):
            result = manager.set_enabled(args.id, args.photo, args.action == 'include')
        elif args.action == 'compute':
            result = manager.compute(args.id)
        else:
            content = manager.export(args.id)
            with args.output.expanduser().open('xb') as f:
                f.write(content)
            result = {'output': str(args.output)}
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
