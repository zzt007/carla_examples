'''
此脚本为 multi_scenario.py 的场景6生成400个硬编码参数的 Python 文件。
主车（queue_vehicle1）、前车（queue_vehicle2）速度可调。
旁车（sur_vehicle_2, sur_vehicle_3）的速度和起始位置（offset_2.x, offset_3.x）分别调整。
总组合数为 5*7*7*7 * 5*4 = 34,300，随机抽样400组。
动态检测缩进，确保参数替换每行独立，保留原始 def main()。
'''

import os
import itertools
import random
import re
import pandas as pd

# 定义输入和输出路径
input_script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\carla_examples\\multi_scenario.py'
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generated_test_cases_6'

# 检查输入文件是否存在
if not os.path.exists(input_script_path):
    print(f"Error: Input file not found at {input_script_path}")
    exit(1)

# 创建输出目录
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 定义因子
V1_SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1]  # queue_vehicle1 速度因子
SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3]  # 其他车辆速度因子
X_OFFSET_FACTORS = [0.5, 0.75, 1.0, 1.25, 1.5]  # 旁车 x 偏移因子
TARGET_NUM_FILES = 400  # 目标生成400个文件
OFFSET_2_X = 3.5  # 固定 offset_2.x
OFFSET_3_X = -3.5  # 固定 offset_3.x

# 场景6参数配置（适配模板中的值）
SCENARIO_6_CONFIG = {
    'speed_vars': [
        ('queue_vehicle1_target_velocity = carla.Vector3D(y=-', '15(\.0)?', ',x=0,z=0)'),
        ('queue_vehicle2_target_velocity = carla.Vector3D(y=', '10(\.0)?', ',x=0,z=0)'),
        ('sur_vehicle_2_target_velocity = carla.Vector3D(y=', '10(\.0)?', ',x=0,z=0)'),
        ('sur_vehicle_3_target_velocity = carla.Vector3D(y=', '10(\.0)?', ',x=0,z=0)')
    ],
    'side_vars': [
        ('offset_1 = carla.Location(y=', '0(\.0)?', ',x=3.8,z=0)'),
        ('offset_2 = carla.Location(y=', '0(\.0)?', ',x=-7.6,z=0)'),
        ('offset_3 = carla.Location(y=', '20(\.0)?', ',x=0,z=0)')
    ],
    'scenario_start': "elif args.scenario == '6':",
    'scenario_end': "elif args.scenario == '7':"
}

# 读取 multi_scenario.py 内容
with open(input_script_path, 'r', encoding='utf-8') as f:
    template_content = f.read()

# 检测场景6代码块的范围
start_marker = SCENARIO_6_CONFIG['scenario_start']
end_marker = SCENARIO_6_CONFIG['scenario_end']
start_index = template_content.find(start_marker)
end_index = template_content.find(end_marker)
if start_index == -1 or end_index == -1:
    print("Error: Could not find scenario 6 code block in multi_scenario.py")
    exit(1)
scenario_code = template_content[start_index:end_index]

# 查找第一行参数的缩进
lines = scenario_code.splitlines()
indent_pattern = r'^\s+'
indent = ''
for line in lines:
    if 'queue_vehicle1_target_velocity' in line:
        match = re.match(indent_pattern, line)
        if match:
            indent = match.group(0)
        break
if not indent:
    print("Warning: Could not detect indentation, using 12 spaces")
    indent = '            '

print(f"Detected indentation: '{indent}' (length: {len(indent)})")
print("Scenario 6 code block:")
print(scenario_code)

# 生成参数组合
speed_combinations = list(itertools.product(V1_SPEED_FACTORS, SPEED_FACTORS, SPEED_FACTORS, SPEED_FACTORS))  # 5*7*7*7 = 1715
side_combinations = [(x2, x3) for x2, x3 in itertools.product(X_OFFSET_FACTORS, X_OFFSET_FACTORS) if x2 != x3]  # 5*4 = 20
total_combinations = len(speed_combinations) * len(side_combinations)  # 1715 * 20 = 34,300
print(f"Total possible combinations for scenario 6: {total_combinations}")

# 随机抽样400组
all_combinations = list(itertools.product(speed_combinations, side_combinations))
if len(all_combinations) < TARGET_NUM_FILES:
    print(f"Error: Only {len(all_combinations)} combinations available, less than {TARGET_NUM_FILES}")
    exit(1)
selected_combinations = random.sample(all_combinations, TARGET_NUM_FILES)

# 保存参数记录
params_data = []

# 为场景6生成文件
for idx, ((v1_speed_factor, v2_speed_factor, sur2_speed_factor, sur3_speed_factor), (x2_factor, x3_factor)) in enumerate(selected_combinations, 1):
    new_content = template_content
    scenario_code = new_content[start_index:end_index]

    # 计算参数值
    v1_speed = max(5.0, min(12.0 * v1_speed_factor, 20.0))  # queue_vehicle1 速度，m/s
    v2_speed = max(5.0, min(10.0 * v2_speed_factor, 20.0))  # queue_vehicle2 速度，m/s
    sur2_speed = max(5.0, min(15.0 * sur2_speed_factor, 20.0))  # sur_vehicle_2 速度，m/s
    sur3_speed = max(5.0, min(15.0 * sur3_speed_factor, 20.0))  # sur_vehicle_3 速度，m/s
    offset_2_y = -20.0 * x2_factor  # offset_2.y，m
    offset_3_y = -20.0 * x3_factor  # offset_3.y，m

    # 替换速度参数
    for i, (var_prefix, base_value, var_suffix) in enumerate(SCENARIO_6_CONFIG['speed_vars']):
        value = [v1_speed, v2_speed, sur2_speed, sur3_speed][i]
        if i == 0:  # target_speed 保留一位小数
            value = round(value, 1)
        else:
            value = value
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{value:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_6_test_case_{idx:03d}.py")
            print("Current scenario code block:")
            print(scenario_code)
            for line in scenario_code.splitlines():
                if any(k in line for k in ['queue_vehicle1_target_velocity', 'queue_vehicle2_target_velocity', 'sur_vehicle_2_target_velocity', 'sur_vehicle_3_target_velocity']):
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换旁车位置（保持 offset_1 不变）
    for i, (var_prefix, base_value, var_suffix) in enumerate(SCENARIO_6_CONFIG['side_vars']):
        if i == 0:  # offset_1 不变
            continue
        value = [offset_2_y, offset_3_y][i - 1]
        x_value = OFFSET_2_X if i == 1 else OFFSET_3_X
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{value:.1f}, x={x_value:.1f}, z=0)\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_6_test_case_{idx:03d}.py")
            print("Current scenario code block:")
            print(scenario_code)
            for line in scenario_code.splitlines():
                if 'offset_2' in line or 'offset_3' in line:
                    print(f"Found line: '{line}'")
            # 调试信息
            print(f"Searching for pattern in line:")
            for line in scenario_code.splitlines():
                print(f"Line: '{line}'")
                if re.search(old_pattern, line):
                    print(f"Match found in line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 更新场景代码
    new_content = new_content[:start_index] + scenario_code + new_content[end_index:]

    # 修改 def main()，确保 default 为 '6'
    main_default_pattern = r"(argparser\.add_argument\([^)]*default\s*=\s*['\"])[0-6](['\"][^)]*\))"
    new_main_default = r"\g<1>6\g<2>"
    if not re.search(main_default_pattern, new_content):
        print(f"Warning: Could not find default value in main() for scenario_6_test_case_{idx:03d}.py")
    new_content = re.sub(main_default_pattern, new_main_default, new_content, count=1)

    # 写入新文件
    output_filename = os.path.join(output_dir, f'scenario_6_test_case_{idx:03d}.py')
    with open(output_filename, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Generated {output_filename} with v1_speed={v1_speed:.1f}, v2_speed={v2_speed:.1f}, sur2_speed={sur2_speed:.1f}, sur3_speed={sur3_speed:.1f}, offset_2.y={offset_2_y:.1f}, offset_3.y={offset_3_y:.1f}")

    # 记录参数
    params_data.append({
        'file': f'scenario_6_test_case_{idx:03d}.py',
        'queue_vehicle1_speed_ms': v1_speed,
        'queue_vehicle2_speed_ms': v2_speed,
        'sur_vehicle2_speed_ms': sur2_speed,
        'sur_vehicle3_speed_ms': sur3_speed,
        'offset_2_y': offset_2_y,
        'offset_3_y': offset_3_y
    })

# 保存参数到 Excel
df = pd.DataFrame(params_data)
df.to_excel(os.path.join(output_dir, 'scenario_6_params.xlsx'), index=False)
print("Parameters saved to scenario_6_params.xlsx")

print("Scenario 6 processing completed.")