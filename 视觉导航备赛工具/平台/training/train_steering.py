#!/usr/bin/env python3
"""Train the vendor-compatible steering CNN on a prepared, portable dataset.

The result is a standalone state dictionary, not a vehicle control program.
No ROS, camera, serial device or network interface is opened by this script.
"""
import argparse
import csv
import hashlib
import json
import math
import random
import time
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset


class AutoPilot(nn.Module):
    """Same layer names and shapes as DEEPBOT's 2025 driver.py."""
    def __init__(self):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(), nn.Flatten())
        self.fc = nn.Sequential(nn.Linear(48 * 12 * 17, 100), nn.ReLU(),
                                nn.Linear(100, 50), nn.ReLU(), nn.Linear(50, 1))

    def forward(self, image):
        return self.fc(self.cnn(image))


def preprocess_rgb(image):
    """Shared image transform for training and vehicle-side preview."""
    resampling = Image.Resampling.BILINEAR if hasattr(Image, 'Resampling') else Image.BILINEAR
    image = image.convert('RGB').resize((160, 120), resampling)
    pixels = np.asarray(image, dtype=np.float32) / 255.0
    pixels = (pixels - np.array([.485, .456, .406], dtype=np.float32)) / np.array([.229, .224, .225], dtype=np.float32)
    return torch.from_numpy(pixels.transpose(2, 0, 1).copy())


class DrivingDataset(Dataset):
    def __init__(self, root, manifest):
        self.root = Path(root).resolve()
        self.samples = []
        seen = set()
        with (self.root / manifest).open(encoding='utf-8-sig', newline='') as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames or not {'image_path', 'steering'}.issubset(reader.fieldnames):
                raise ValueError(f'{manifest} 缺少必要字段')
            for row in reader:
                path = (self.root / row['image_path']).resolve()
                label = float(row['steering'])
                if self.root not in path.parents or not path.is_file() or path in seen or not math.isfinite(label):
                    raise ValueError(f'{manifest} 包含无效、重复或越界的样本')
                self.samples.append((path, label))
                seen.add(path)
        if not self.samples:
            raise ValueError(f'{manifest} 没有样本')

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        path, steering = self.samples[index]
        with Image.open(path) as image:
            pixels = preprocess_rgb(image)
        return pixels, torch.tensor([steering], dtype=torch.float32)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for part in iter(lambda: source.read(1024 * 1024), b''):
            h.update(part)
    return h.hexdigest()


def evaluate(model, loader, device):
    model.eval()
    abs_error, squared_error, count = 0.0, 0.0, 0
    with torch.no_grad():
        for images, target in loader:
            error = model(images.to(device)) - target.to(device)
            abs_error += error.abs().sum().item()
            squared_error += error.square().sum().item()
            count += target.numel()
    return {'mae': abs_error / count, 'mse': squared_error / count}


def run(args):
    if args.epochs < 1 or args.batch_size < 1 or args.workers < 0 or not math.isfinite(args.lr) or args.lr <= 0:
        raise ValueError('训练参数无效')
    root = args.dataset.expanduser().resolve()
    output = args.output.expanduser().absolute()
    if output.exists():
        raise ValueError('训练输出目录已存在，请使用新的目录')
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise ValueError('未检测到 CUDA，未启动训练')
    if args.device == 'mps' and not torch.backends.mps.is_available():
        raise ValueError('未检测到 Apple GPU 加速，未启动训练')
    train = DrivingDataset(root, 'train.csv')
    validation = DrivingDataset(root, 'validation.csv')
    source_meta = {}
    if (root / 'metadata.json').is_file():
        source_meta = json.loads((root / 'metadata.json').read_text(encoding='utf-8'))
        if not isinstance(source_meta, dict):
            raise ValueError('数据集元信息格式无效')
    if {p for p, _ in train.samples} & {p for p, _ in validation.samples}:
        raise ValueError('训练和验证集存在重复图片')
    test = DrivingDataset(root, 'test.csv') if (root / 'test.csv').is_file() else None
    if test and ({p for p, _ in test.samples} & {p for p, _ in train.samples + validation.samples}):
        raise ValueError('测试集与训练集或验证集存在重复图片')
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if args.device == 'cuda':
        torch.cuda.manual_seed_all(args.seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    device = torch.device(args.device)
    train_loader = DataLoader(train, batch_size=args.batch_size, shuffle=True, num_workers=args.workers,
                              pin_memory=args.device == 'cuda')
    val_loader = DataLoader(validation, batch_size=args.batch_size, shuffle=False, num_workers=args.workers,
                            pin_memory=args.device == 'cuda')
    model = AutoPilot().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()
    labels = [target for _, target in train.samples]
    val_labels = [target for _, target in validation.samples]
    avg = sum(labels) / len(labels)
    baseline = {'predict_zero_mse': sum(v * v for v in val_labels) / len(val_labels),
                'predict_train_mean_mse': sum((v - avg) ** 2 for v in val_labels) / len(val_labels)}
    output.mkdir(parents=True, exist_ok=False)
    metadata = {'format': 'DEEPBOT 2025 AutoPilot state_dict', 'train_samples': len(train),
                'validation_samples': len(validation), 'label': source_meta.get('label', 'steering; source unspecified'),
                'dataset_source': source_meta.get('source', 'unspecified'),
                'input': 'RGB 160x120, ImageNet mean and std', 'seed': args.seed,
                'epochs': args.epochs, 'batch_size': args.batch_size, 'learning_rate': args.lr,
                'train_csv_sha256': digest(root / 'train.csv'),
                'validation_csv_sha256': digest(root / 'validation.csv'),
                'torch_version': torch.__version__, 'device': str(device),
                'gpu': torch.cuda.get_device_name(0) if args.device == 'cuda' else ('Apple Metal' if args.device == 'mps' else None),
                'baseline': baseline,
                'note': 'A time-separated segment of one recording does not substitute for an independent real-track test.'}
    if test:
        metadata['test_samples'] = len(test)
        metadata['test_csv_sha256'] = digest(root / 'test.csv')
    (output / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    best = float('inf')
    started = time.monotonic()
    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss, count = 0.0, 0
        for images, target in train_loader:
            images, target = images.to(device), target.to(device)
            optimizer.zero_grad(set_to_none=True)
            prediction = model(images)
            loss = loss_fn(prediction, target)
            if not torch.isfinite(loss):
                raise ValueError('训练损失不是有限数，已停止')
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * target.numel()
            count += target.numel()
        metrics = evaluate(model, val_loader, device)
        record = {'epoch': epoch, 'train_mse': total_loss / count, 'validation_mse': metrics['mse'],
                  'validation_mae': metrics['mae'], 'elapsed_seconds': round(time.monotonic() - started, 2)}
        if not all(math.isfinite(record[key]) for key in ('train_mse', 'validation_mse', 'validation_mae')):
            raise ValueError('验证结果不是有限数，已停止')
        if metrics['mse'] < best:
            best = metrics['mse']
            torch.save(model.state_dict(), output / 'autopilot.pth')
            metadata['best_epoch'] = epoch
            metadata['best_validation_mse'] = best
            metadata['model_sha256'] = digest(output / 'autopilot.pth')
            (output / 'metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        with (output / 'metrics.jsonl').open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, allow_nan=False) + '\n')
        print(json.dumps(record, ensure_ascii=False), flush=True)
    if test:
        model.load_state_dict(torch.load(str(output / 'autopilot.pth'), map_location='cpu', weights_only=True))
        test_loader = DataLoader(test, batch_size=args.batch_size, shuffle=False, num_workers=args.workers,
                                 pin_memory=args.device == 'cuda')
        metrics = evaluate(model, test_loader, device)
        test_labels = [target for _, target in test.samples]
        report = {'mae': metrics['mae'], 'mse': metrics['mse'], 'samples': len(test),
                  'predict_zero_mse': sum(v * v for v in test_labels) / len(test_labels),
                  'predict_train_mean_mse': sum((v - avg) ** 2 for v in test_labels) / len(test_labels),
                  'note': 'Held-out data must be assessed against its source; no real-track guarantee.'}
        if not all(math.isfinite(report[key]) for key in ('mae', 'mse', 'predict_zero_mse', 'predict_train_mean_mse')):
            raise ValueError('测试结果不是有限数')
        (output / 'test_metrics.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({'test': report}, ensure_ascii=False), flush=True)
    return metadata


def main():
    parser = argparse.ArgumentParser(description='训练厂家兼容的方向预测模型；不控制车辆')
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=20)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--lr', type=float, default=.001)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--seed', type=int, default=2026)
    parser.add_argument('--device', choices=('cuda', 'mps', 'cpu'), default='cuda')
    args = parser.parse_args()
    try:
        run(args)
    except (ValueError, OSError, csv.Error) as error:
        parser.exit(2, '训练失败：' + str(error) + '\n')


if __name__ == '__main__':
    main()
