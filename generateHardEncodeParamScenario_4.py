'''
此脚本为 multi_scenario.py 的场景4生成400个硬编码参数的 Python 文件。
主车（queue_vehicle1）速度因子最大1.1避免换道滑移，前车（queue_vehicle2, queue_vehicle3）速度和间距可调。
旁车（sur_vehicle）的速度和起始位置（offset_3.y 非零）分别调整。
总组合数为 5*7*7*7 * 7*7 * 4 = 336,140，随机抽样400组。
动态检测缩进，确保参数替换每行独立，保留原始 def main()。
'''

import os
import itertools
import random
import re
import pandas as pd

# 定义输入和输出路径
input_script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\carla_examples\\multi_scenario.py'
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generated_test_cases_4'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 定义因子
V1_SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1]  # queue_vehicle1 速度因子，最大1.1
SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3]  # 其他车辆速度因子
DISTANCE_FACTORS = [0.5, 0.6, 0.8, 1.0, 1.1, 1.2, 1.5]  # 间距因子
SIDE_OFFSETS = [-5.0, -3.0, 3.0, 5.0]  # 旁车位置偏移，确保 y 非零
TARGET_NUM_FILES = 400  # 目标生成400个文件

# 场景4参数配置
SCENARIO_4_CONFIG = {
    'speed_vars': [
        ('target_speed = ', '12(\.0)?', ' * 3.6'),
        ('queue_vehicle2_target_velocity = carla.Vector3D(x=', '10(\.0)?', ',y=0,z=0)'),
        ('queue_vehicle3_target_velocity = carla.Vector3D(x=', '10(\.0)?', ',y=0,z=0)'),
        ('sur_vehicle_target_velocity = carla.Vector3D(x=', '15(\.0)?', ',y=0,z=0)')
    ],
    'distance_vars': [
        ('offset_1 = carla.Location(x=', '20(\.0)?', ',y=3.5,z=0)'),
        ('offset_2 = carla.Location(x=', '20(\.0)?', ',y=0,z=0)')
    ],
    'side_var': ('offset_3 = carla.Location(x=-20,y=', '3\.5', ',z=0)'),
    'scenario_start': "elif args.scenario == '4':",
    'scenario_end': "elif args.scenario == '5':"
}

# 读取 multi_scenario.py 内容
with open(input_script_path, 'r', encoding='utf-8') as f:
    template_content = f.read()

# 检测场景4代码块的缩进
start_marker = SCENARIO_4_CONFIG['scenario_start']
end_marker = SCENARIO_4_CONFIG['scenario_end']
start_index = template_content.find(start_marker)
end_index = template_content.find(end_marker)
if start_index == -1 or end_index == -1:
    print("Error: Could not find scenario 4 code block in multi_scenario.py")
    exit(1)
scenario_code = template_content[start_index:end_index]

# 查找第一行参数的缩进
lines = scenario_code.splitlines()
indent_pattern = r'^\s+'
indent = ''
for line in lines:
    if 'target_speed' in line:
        match = re.match(indent_pattern, line)
        if match:
            indent = match.group(0)
        break
if not indent:
    print("Warning: Could not detect indentation, using 12 spaces")
    indent = '            '

print(f"Detected indentation: '{indent}' (length: {len(indent)})")

# 生成参数组合
speed_combinations = list(itertools.product(V1_SPEED_FACTORS, SPEED_FACTORS, SPEED_FACTORS, SPEED_FACTORS))  # 5*7*7*7 = 1715
distance_combinations = list(itertools.product(DISTANCE_FACTORS, DISTANCE_FACTORS))  # 7*7 = 49
side_combinations = list(SIDE_OFFSETS)  # 4
total_combinations = len(speed_combinations) * len(distance_combinations) * len(side_combinations)  # 1715 * 49 * 4 = 336140
print(f"Total possible combinations for scenario 4: {total_combinations}")

# 随机抽样400组
all_combinations = list(itertools.product(speed_combinations, distance_combinations, side_combinations))
if len(all_combinations) < TARGET_NUM_FILES:
    print(f"Error: Only {len(all_combinations)} combinations available, less than {TARGET_NUM_FILES}")
    exit(1)
selected_combinations = random.sample(all_combinations, TARGET_NUM_FILES)

# 保存参数记录
params_data = []

# 为场景4生成文件
for idx, ((v1_speed_factor, v2_speed_factor, v3_speed_factor, sur_speed_factor), (dist1_factor, dist2_factor), side_offset) in enumerate(selected_combinations, 1):
    new_content = template_content
    scenario_code = new_content[start_index:end_index]

    # 计算参数值
    v1_speed = max(5.0, min(12.0 * v1_speed_factor, 20.0))  # queue_vehicle1 速度，km/h
    v2_speed = max(5.0, min(10.0 * v2_speed_factor, 20.0))
    # 为确保v1能换道，尽可能让v1车速快
    v1_speed = 1.1* max(v1_speed, v2_speed)  # queue_vehicle1 速度，m/
    # queue_vehicle2 速度，m/s
    v3_speed = v2_speed  # queue_vehicle3 速度，m/s
    sur_speed = max(5.0, min(15.0 * sur_speed_factor, 20.0))  # sur_vehicle 速度，m/s
    dist1 = max(10.0, min(20.0 * dist1_factor, 50.0))  # offset_1.x，m
    dist2 = max(10.0, min(20.0 * dist2_factor, 50.0))  # offset_2.x，m

    # 替换速度参数
    for i, (var_prefix, base_value, var_suffix) in enumerate(SCENARIO_4_CONFIG['speed_vars']):
        value = [v1_speed, v2_speed, v3_speed, sur_speed][i]
        if i == 0:  # target_speed 保留一位小数
            value = round(value, 1)
        else:
            value = value
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{value:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_4_test_case_{idx:03d}.py")
            for line in scenario_code.splitlines():
                if any(k in line for k in ['target_speed', 'queue_vehicle2_target_velocity', 'queue_vehicle3_target_velocity', 'sur_vehicle_target_velocity']):
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换间距
    for i, (var_prefix, base_value, var_suffix) in enumerate(SCENARIO_4_CONFIG['distance_vars']):
        value = [dist1, dist2][i]
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{value:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_4_test_case_{idx:03d}.py")
            for line in scenario_code.splitlines():
                if 'offset_1' in line or 'offset_2' in line:
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换旁车位置
    var_prefix, base_value, var_suffix = SCENARIO_4_CONFIG['side_var']
    old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
    new_line = f"{indent}{var_prefix}{side_offset:.1f}{var_suffix}\n"
    if not re.search(old_pattern, scenario_code, re.MULTILINE):
        print(f"Warning: Could not find pattern '{old_pattern}' in scenario_4_test_case_{idx:03d}.py")
        for line in scenario_code.splitlines():
            if 'offset_3' in line:
                print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 更新场景代码
    new_content = new_content[:start_index] + scenario_code + new_content[end_index:]

    # 修改 def main()，仅更改 default='6' 为 default='4'
    main_default_pattern = r"(argparser\.add_argument\([^)]*default\s*=\s*['\"])6(['\"][^)]*\))"
    new_main_default = r"\g<1>4\g<2>"
    if not re.search(main_default_pattern, new_content):
        print(f"Warning: Could not find default='6' in main() for scenario_4_test_case_{idx:03d}.py")
    new_content = re.sub(main_default_pattern, new_main_default, new_content, count=1)

    # 写入新文件
    output_filename = os.path.join(output_dir, f'scenario_4_test_case_{idx:03d}.py')
    with open(output_filename, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Generated {output_filename} with v1_speed={v1_speed/3.6:.1f}m/s, v2_speed={v2_speed:.1f}, v3_speed={v3_speed:.1f}, sur_speed={sur_speed:.1f}, dist1={dist1:.1f}, dist2={dist2:.1f}, offset_3.y={side_offset:.1f}")

    # 记录参数
    params_data.append({
        'file': f'scenario_4_test_case_{idx:03d}.py',
        'queue_vehicle1_speed_ms': v1_speed / 3.6,
        'queue_vehicle2_speed_ms': v2_speed,
        'queue_vehicle3_speed_ms': v3_speed,
        'sur_vehicle_speed_ms': sur_speed,
        'offset_1_x': dist1,
        'offset_2_x': dist2,
        'offset_3_y': side_offset
    })

# 保存参数到 Excel
df = pd.DataFrame(params_data)
df.to_excel(os.path.join(output_dir, 'scenario_4_params.xlsx'), index=False)
print("Parameters saved to scenario_4_params.xlsx")

print("Scenario 4 processing completed.")