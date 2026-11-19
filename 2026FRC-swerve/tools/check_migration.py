#!/usr/bin/env python3
"""移植靜態核對。不是 Java 編譯或硬體驗證。"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

root=Path(__file__).resolve().parents[1]
baseline=json.loads((root/'docs/migration/parameter-baseline.json').read_text())
for name, original in baseline.items():
    current=(root/name).read_text()
    if name.endswith('/Constants.java'):
        def constants(text):
            return Counter((name,re.sub(r'\s+','',expression)) for name,expression in
                re.findall(r'public static final [\w.]+ (\w+)\s*=\s*([^;]+);',text))
        before = constants(original['content'])
        after = constants(current)
        # 使用者取消分軸倍率：只允許這三個常數合併為原本共同的 0.005。
        old_scales = ['kForwardSpeedScale', 'kStrafeSpeedScale', 'kRotationSpeedScale']
        for name in old_scales:
            assert before[(name, '0.005')] == 1, f'倍率基準不符：{name}'
            assert not any(key[0] == name for key in after), f'舊分軸倍率仍存在：{name}'
            del before[(name, '0.005')]
        assert after[('kDriverSpeedScale', '0.005')] == 1, '共用手把倍率必須保留 0.005'
        missing = before - after
        assert not missing, f'既有參數改變：{missing}'
    else:
        assert hashlib.sha256(current.encode()).hexdigest()==original['sha256'], f'基準改變：{name}'
versions={'REVLib':'2026.0.5','Phoenix6':'26.3.0','AdvantageKit':'26.0.2',
          'photonlib':'v2026.3.4','maple-sim':'0.4.0-beta','PathplannerLib':'2026.1.2'}
for filename in (root/'vendordeps').glob('*.json'):
    data=json.loads(filename.read_text()); name=data['name']
    if name in versions:assert data['version']==versions[name],(name,data['version'])
    assert '2027' not in data['version'],filename
assert 'version "2026.2.1"' in (root/'build.gradle').read_text()
robot='\n'.join(p.read_text() for p in (root/'src/main/java/frc/robot').rglob('*.java'))
for banned in ['SwerveRequest','TalonFX','CANcoder','getKraken','k2025Reefscape','new Notifier','com.team11855']:
    assert banned not in robot, f'實作仍含 {banned}'
assert 'k2026RebuiltWelded' in robot
assert not list((root/'src/main/deploy').rglob('navgrid*.json')), '新增地圖需另行核對，不可宣稱空圖有效'
rev=(root/'src/main/java/frc/robot/subsystems/drive/DriveIORev.java').read_text()
for corner in ['FrontLeft','FrontRight','RearLeft','RearRight']:
    assert f'k{corner}DrivingInverted' in rev
assert 'drivingConfig.inverted(drivingInverted)' in (root/'src/main/java/frc/robot/subsystems/MAXSwerveModule.java').read_text()
# Every local import must resolve, allowing nested types, wildcards and generated AutoLog classes.
java_root=root/'src/main/java'
for path in java_root.rglob('*.java'):
    text=path.read_text()
    for imported in re.findall(r'^import (?:static )?(frc\.[\w.*]+);',text,re.M):
        parts=imported.split('.')
        assert any((java_root/('/'.join(parts[:i])+'.java')).exists() or
                   (java_root/('/'.join(parts[:i]))).is_dir() and imported.endswith('.*')
                   for i in range(2,len(parts)+1)), f'{path}: import {imported}'
print('靜態核對通過：既有常數（手把倍率已依指示合併）、REV 設定、隊號、依賴版本、輪組映射、禁止硬體引用與本地 import。')
print('尚未編譯、執行 Java 測試、模擬或部署；靜態核對不能證明 Java 型別與執行行為正確。')
