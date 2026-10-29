#!/usr/bin/env python3
"""Create a new, whitelist-only vehicle staging archive without deploying it."""
import argparse
import csv
import hashlib
import io
import tarfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FILES = [
    'automation/__init__.py', 'automation/base_protocol.py', 'automation/command.py',
    'automation/barrier_avoidance.py', 'automation/barrier_detection.py',
    'automation/detector.py', 'automation/feedback.py', 'automation/model_crosswalk.py',
    'automation/race.py', 'automation/range_safety.py',
    'automation/realtime.py', 'automation/steering_candidate.py', 'automation/supervisor.py',
    'automation/voice_protocol.py', 'automation/yolo.py',
    'ros2/automatic_race.py', 'ros2/barrier_avoidance.py', 'ros2/barrier_detector.py',
    'ros2/camera_node.py', 'ros2/command_gate.py', 'ros2/range_safety.py',
    'ros2/deepbot_base.py', 'ros2/openvino_preview.py', 'ros2/run_foxy.sh',
    'ros2/serial_voice_player.py', 'ros2/steering_candidate.py', 'ros2/steering_preview.py',
    'ros2/telemetry_bridge.py', 'ros2/vehicle_preview.sh',
    'tools/steering_gate_smoke.py', 'tools/steering_ros2_smoke.py',
    'tools/ros2_model_smoke.py', 'tools/ros2_barrier_smoke.py', 'tools/visual_safety_replay.py',
    'tools/inspect_barrier.py',
    'training/train_steering.py', 'data/perception.example.json', 'data/perception.lab-example.json',
    'data/crosswalk-model.example.json', 'data/barrier.example.json',
    'data/avoidance.example.json', 'data/range-safety.example.json',
]
README = '''视觉导航实时控制联调包

此包不自动启动程序，也不覆盖车上已有目录。先解压到小车上的新目录，并核对 MANIFEST.sha256。
data/perception.lab-example.json 保持未标定；不能把它作为正式赛道参数。

仅在 ROS 2 本机隔离域回放，以下命令不操作实车底盘：
ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 bash ros2/run_foxy.sh python3 tools/steering_gate_smoke.py --model "$PWD/data/models/steering.pth" --image "$PWD/data/samples/reference.jpg"
ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 bash ros2/run_foxy.sh python3 tools/ros2_model_smoke.py
ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 bash ros2/run_foxy.sh python3 tools/ros2_barrier_smoke.py
python3 tools/visual_safety_replay.py

小车需要 ROS 2 Foxy、消息包、PyTorch、NumPy、Pillow、OpenCV 和串口依赖。
该回放使用历史图片和模拟安全信号、急停、里程计和末端指令缓冲，并不验证实车避障或制动。
人行道模型回放使用模拟模型结果，现场模型 XML/BIN 应从小车现有安装读取。挡板识别、避障和距离许可的示例参数均未标定。
仲裁和底盘适配器默认均使用 TwistStamped，需同一 ROS 时钟，不能与旧版 Twist 接口混用。
底盘接口默认只读；要运行实体运动必须先现场核对方向、独立安全监测、急停、里程计、速度及失联保护。
软件超时依赖程序运行，不能代替底盘固件独立看门狗和实体急停。
'''


def add_data(archive, name, content, mode=0o644):
    info = tarfile.TarInfo(name)
    info.size, info.mode, info.mtime = len(content), mode, int(time.time())
    archive.addfile(info, io.BytesIO(content))


def main():
    parser = argparse.ArgumentParser(description='生成单独的车端实时控制联调包')
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.expanduser().absolute()
    if output.exists():
        parser.error('输出文件已存在')
    dataset = args.dataset.expanduser().resolve()
    with (dataset / 'test.csv').open(encoding='utf-8-sig', newline='') as handle:
        first = next(csv.DictReader(handle), None)
    if not first or not first.get('image_path'):
        parser.error('测试集中没有样本')
    sample = (dataset / first['image_path']).resolve()
    model = args.model.expanduser().resolve()
    if dataset not in sample.parents or not sample.is_file() or not model.is_file():
        parser.error('模型或测试图片不存在')
    contents = {}
    for name in FILES:
        path = ROOT / name
        if not path.is_file() or path.is_symlink():
            parser.error('缺少程序文件: ' + name)
        contents[name] = path.read_bytes()
    contents['data/models/steering.pth'] = model.read_bytes()
    contents['data/samples/reference.jpg'] = sample.read_bytes()
    contents['README-联调.txt'] = README.encode('utf-8')
    checksum = ''.join(hashlib.sha256(content).hexdigest() + '  ' + name + '\n'
                       for name, content in sorted(contents.items())).encode('utf-8')
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with output.open('xb') as target:
            with tarfile.open(fileobj=target, mode='w:gz') as archive:
                for name, content in sorted(contents.items()):
                    add_data(archive, name, content, 0o755 if name.endswith('.sh') else 0o644)
                add_data(archive, 'MANIFEST.sha256', checksum)
    except Exception:
        output.unlink(missing_ok=True)
        raise
    print('已创建联调包：' + str(output))
    print('文件数：' + str(len(contents)) + '；尚未部署到小车。')


if __name__ == '__main__':
    main()
