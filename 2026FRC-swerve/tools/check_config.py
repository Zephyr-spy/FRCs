#!/usr/bin/env python3
"""只讀 Java 原始碼的待設定檢查；不載入 Java、不編譯、不連機器人。"""
import argparse
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'src/main/java/frc/robot/config/RobotSettings.java'
PATTERN = re.compile(r'@Pending\("((?:[^"\\]|\\.)*)"\)\s*public\s+static\s+final\s+(double|int|boolean|String)\s+(\w+)\s*=\s*([^;]+);')
POSITIVE = {'heightMeters','massKg','moiKgMetersSquared','bumperLengthMeters','bumperWidthMeters',
            'cogHeightMeters','wheelCOF','steerReduction','steerInertia','width','height','fps',
            'p'}
NONNEGATIVE = {'driveFrictionVolts','steerFrictionVolts','latencyMs','i','d'}

def warnings(source):
    results = []
    for match in PATTERN.finditer(source):
        description, kind, name, expression = match.groups()
        expression = expression.strip()
        reason = None
        try:
            if kind == 'String':
                value = json.loads(expression)
                if not value.strip(): reason = '待設定'
            elif kind == 'boolean':
                if expression == 'false': reason = '待確認'
                elif expression != 'true': raise ValueError()
            else:
                value = float(expression)
                if not math.isfinite(value): reason = '待設定'
                elif name in POSITIVE and value <= 0: reason = '數值需大於 0'
                elif name in NONNEGATIVE and value < 0: reason = '數值不可為負'
                elif name == 'count' and value != 1: reason = '此介面僅接受單相機，count 必須為 1'
                elif name == 'diagonalFovDegrees' and not 0 < value < 180: reason = '視角需介於 0 與 180 度'
                elif kind == 'int' and (value < 0 or value != int(value)): reason = '待設定整數'
        except (ValueError, TypeError):
            reason = '待設定' if expression == 'Double.NaN' else '無法靜態解析，請使用明確常數值'
        if reason:
            line = source.count('\n', 0, match.start(4)) + 1
            column = match.start(4) - source.rfind('\n', 0, match.start(4))
            results.append((line, column, f'{reason}：{name} — {description}'))
    if source.count('@Pending(') != len(list(PATTERN.finditer(source))):
        results.append((1, 1, '有無法解析的 @Pending 宣告，請檢查標記格式'))
    return results

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', nargs='?', type=Path, default=DEFAULT)
    args = parser.parse_args()
    path = args.file.resolve()
    entries = warnings(path.read_text(encoding='utf-8'))
    for line, column, message in entries:
        print(f'{path}:{line}:{column}: warning: {message}')
    print(f'待設定檢查完成：{len(entries)} 項警告；未編譯、未連接機器人。')

if __name__ == '__main__': main()
