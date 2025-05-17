'''
此脚本基于 batchTest2.py，生成 364 个硬编码 Python 文件，每个文件内容基于 batchProcessing2_3cars.py。
硬编码参数：sur_vehicle1 的 offset_x, offset_y 和 sur_vehicle2 的 velocity_x。
移除命令行传参逻辑，使每个文件可独立运行。
参数组合：offset_x (-4.0 到 8.0), offset_y (-13.0 到 14.0), velocity_x (5.0 到 16.0 循环分配)。
输出文件：test_case_001.py 到 test_case_364.py。
保存参数到 test_case_params.xlsx。
'''

import os
import itertools
import numpy as np
import pandas as pd
import re
import hashlib

# 定义输入和输出路径
input_script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\carla_examples\\batchProcessing2_3cars.py'
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generated_test_cases_2'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 定义参数范围
vehicle1_offset_x_min, vehicle1_offset_x_max = -4.0, 8.0  # sur_vehicle1 offset_x
vehicle1_offset_y_min, vehicle1_offset_y_max = -13.0, 14.0  # sur_vehicle1 offset_y
vehicle2_velocity_x_min, vehicle2_velocity_x_max = 5.0, 16.0  # sur_vehicle2 velocity_x
step = 1.0  # 步长

# 生成 sur_vehicle1 的偏移组合
sur_vehicle1_offsets = [
    [x, y]
    for x in np.arange(vehicle1_offset_x_min, vehicle1_offset_x_max + step, step)
    for y in np.arange(vehicle1_offset_y_min, vehicle1_offset_y_max + step, step)
]

# 生成 sur_vehicle2 的速度可取值
sur_vehicle2_velocity_optional = [
    [x]
    for x in np.arange(vehicle2_velocity_x_min, vehicle2_velocity_x_max + step, step)
]

# 生成参数组合（velocity_x 循环分配）
combinations = []
for i, offset in enumerate(sur_vehicle1_offsets):
    velocity_idx = i % len(sur_vehicle2_velocity_optional)  # 循环分配 velocity_x
    velocity = sur_vehicle2_velocity_optional[velocity_idx]
    combinations.append({
        'sur1_vehicle_x': offset[0],
        'sur1_vehicle_y': offset[1],
        'sur2_vehicle_speed_lon': velocity[0]
    })

print(f"Total combinations: {len(combinations)}")

# 读取 batchProcessing2_3cars.py 内容
with open(input_script_path, 'r', encoding='utf-8') as f:
    template_content = f.read()

# 检测缩进
indent_pattern = r'^\s+SUR1_VEHICLE_X\s*=\s*params\[0\]'
lines = template_content.splitlines()
indent = '    '  # 默认 4 空格
for line in lines:
    if 'SUR1_VEHICLE_X = params[0]' in line:
        match = re.match(indent_pattern, line)
        if match:
            indent = match.group(0).split('SUR1_VEHICLE_X')[0]
        break
print(f"Detected indentation: '{indent}' (length: {len(indent)})")

# 检查重复组合
param_hashes = set()
duplicate_files = []

# 保存参数记录
params_data = []

# 生成文件
for idx, params in enumerate(combinations, 1):
    new_content = template_content

    # 提取参数
    sur1_vehicle_x = params['sur1_vehicle_x']
    sur1_vehicle_y = params['sur1_vehicle_y']
    sur2_vehicle_speed_lon = params['sur2_vehicle_speed_lon']

    # 检查重复
    param_tuple = (sur1_vehicle_x, sur1_vehicle_y, sur2_vehicle_speed_lon)
    param_hash = hashlib.md5(str(param_tuple).encode()).hexdigest()
    if param_hash in param_hashes:
        duplicate_files.append(f"test_case_{idx:03d}.py: {param_tuple}")
    param_hashes.add(param_hash)

    # 替换参数
    replacements = [
        (
            rf"^{indent}SUR1_VEHICLE_X\s*=\s*params\[0\](\s*\n)?",
            f"{indent}SUR1_VEHICLE_X = {sur1_vehicle_x:.1f}\n"
        ),
        (
            rf"^{indent}SUR1_VEHICLE_Y\s*=\s*params\[1\](\s*\n)?",
            f"{indent}SUR1_VEHICLE_Y = {sur1_vehicle_y:.1f}\n"
        ),
        (
            rf"^{indent}SUR2_VEHICLE_SPEED_LON\s*=\s*params\[2\](\s*\n)?",
            f"{indent}SUR2_VEHICLE_SPEED_LON = {sur2_vehicle_speed_lon:.1f}\n"
        )
    ]

    for old_pattern, new_line in replacements:
        if not re.search(old_pattern, new_content, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in test_case_{idx:03d}.py")
            print("Relevant code section:")
            for line in new_content.splitlines():
                if any(k in line for k in ['SUR1_VEHICLE_X', 'SUR1_VEHICLE_Y', 'SUR2_VEHICLE_SPEED_LON']):
                    print(f"Found line: '{line}'")
        new_content = re.sub(old_pattern, new_line, new_content, count=1, flags=re.MULTILINE)

    # 移除命令行逻辑
    command_line_pattern = r"(?s)# 接收来自自动化测试脚本batchTest\.py输出的参数.*?sys\.exit\(0\)\n"
    new_content = re.sub(command_line_pattern, '', new_content, count=1)

    # 写入文件
    output_filename = os.path.join(output_dir, f'test_case_{idx:03d}.py')
    with open(output_filename, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Generated {output_filename} with sur1_vehicle_x={sur1_vehicle_x:.1f}, sur1_vehicle_y={sur1_vehicle_y:.1f}, sur2_vehicle_speed_lon={sur2_vehicle_speed_lon:.1f}")

    # 记录参数
    params_data.append({
        'file': f'test_case_{idx:03d}.py',
        'sur1_vehicle_x': sur1_vehicle_x,
        'sur1_vehicle_y': sur1_vehicle_y,
        'sur2_vehicle_speed_lon': sur2_vehicle_speed_lon
    })

# 检查重复
if duplicate_files:
    print("Warning: Found duplicate parameter combinations:")
    for dup in duplicate_files:
        print(dup)
else:
    print("No duplicate parameter combinations found.")

# 保存参数到 Excel
df = pd.DataFrame(params_data)
df.to_excel(os.path.join(output_dir, 'test_case_params.xlsx'), index=False)
print("Parameters saved to test_case_params.xlsx")

# 打印参数统计
print("\nParameter Statistics:")
for col in df.columns[1:]:
    print(f"{col}: min={df[col].min():.1f}, max={df[col].max():.1f}, unique={len(df[col].unique())}")

print("All tests generated.")