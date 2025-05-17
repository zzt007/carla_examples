'''
此脚本为 multi_scenario.py 的场景5生成400个硬编码参数的 Python 文件。
主车（queue_vehicle1）、前车（queue_vehicle2）速度可调，间距不小于15m。
旁车（sur_vehicle2, sur_vehicle3）的速度和起始位置（offset_2.y, offset_3.y 非零）分别调整。
适配模板中 offset_2 和 offset_3 的实际定义，增强 def main() 匹配。
总组合数为 5*7*7*7 * 7 * 4*4 = 192,080，随机抽样400组。
动态检测缩进，确保参数替换每行独立，保留原始 def main()。
'''

import os
import itertools
import random
import re
import pandas as pd

# 定义输入和输出路径
input_script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\carla_examples\\multi_scenario.py'
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generated_test_cases_5'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 定义因子
V1_SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1]  # queue_vehicle1 速度因子，最大1.1
SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3]  # 其他车辆速度因子
DISTANCE_FACTORS = [0.75, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5]  # 间距因子，确保不小于15m
SIDE_OFFSETS = [-5.0, -3.0, 3.0, 5.0]  # 旁车位置偏移，确保 y 非零
TARGET_NUM_FILES = 400  # 目标生成400个文件

# 场景5参数配置
SCENARIO_5_CONFIG = {
    'speed_vars': [
        ('queue_vehicle1_target_velocity = carla.Vector3D(x=', '12(\.0)?', ',y=0,z=0)'),
        ('queue_vehicle2_target_velocity = carla.Vector3D(x=', '10(\.0)?', ',y=0,z=0)'),
        ('sur_vehicle2_target_velocity = carla.Vector3D(x=', '15(\.0)?', ',y=0,z=0)'),
        ('sur_vehicle3_target_velocity = carla.Vector3D(x=', '15(\.0)?', ',y=0,z=0)')
    ],
    'distance_vars': [
        ('offset_1 = carla.Location(x=', '20(\.0)?', ',y=0,z=0)')
    ],
    'side_vars': [
        ('offset_2 = carla.Location(x=20,y=', '0(\.0)?', ',z=0)'),
        ('offset_3 = carla.Location(x=0,y=', '-4(\.0)?', ',z=0)')
    ],
    'scenario_start': "elif args.scenario == '5':",
    'scenario_end': "elif args.scenario == '6':"
}

# 读取 multi_scenario.py 内容
with open(input_script_path, 'r', encoding='utf-8') as f:
    template_content = f.read()

# 检测场景5代码块的缩进
start_marker = SCENARIO_5_CONFIG['scenario_start']
end_marker = SCENARIO_5_CONFIG['scenario_end']
start_index = template_content.find(start_marker)
end_index = template_content.find(end_marker)
if start_index == -1 or end_index == -1:
    print("Error: Could not find scenario 5 code block in multi_scenario.py")
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

# 生成参数组合
speed_combinations = list(itertools.product(V1_SPEED_FACTORS, SPEED_FACTORS, SPEED_FACTORS, SPEED_FACTORS))  # 5*7*7*7 = 1715
distance_combinations = list(DISTANCE_FACTORS)  # 7
side_combinations = list(itertools.product(SIDE_OFFSETS, SIDE_OFFSETS))  # 4*4 = 16
total_combinations = len(speed_combinations) * len(distance_combinations) * len(side_combinations)  # 1715 * 7 * 16 = 192080
print(f"Total possible combinations for scenario 5: {total_combinations}")

# 随机抽样400组
all_combinations = list(itertools.product(speed_combinations, distance_combinations, side_combinations))
if len(all_combinations) < TARGET_NUM_FILES:
    print(f"Error: Only {len(all_combinations)} combinations available, less than {TARGET_NUM_FILES}")
    exit(1)
selected_combinations = random.sample(all_combinations, TARGET_NUM_FILES)

# 保存参数记录
params_data = []

# 为场景5生成文件
for idx, ((v1_speed_factor, v2_speed_factor, sur2_speed_factor, sur3_speed_factor), dist_factor, (side_offset2, side_offset3)) in enumerate(selected_combinations, 1):
    new_content = template_content
    scenario_code = new_content[start_index:end_index]

    # 计算参数值
    v1_speed = max(5.0, min(12.0 * v1_speed_factor, 20.0))  # queue_vehicle1 速度，m/s
    v2_speed = max(5.0, min(10.0 * v2_speed_factor, 20.0))  # queue_vehicle2 速度，m/s
    sur2_speed = max(5.0, min(15.0 * sur2_speed_factor, 20.0))  # sur_vehicle2 速度，m/s
    sur3_speed = max(5.0, min(15.0 * sur3_speed_factor, 20.0))  # sur_vehicle3 速度，m/s
    dist = max(15.0, min(20.0 * dist_factor, 50.0))  # offset_1.x，m

    # 替换速度参数
    for i, (var_prefix, base_value, var_suffix) in enumerate(SCENARIO_5_CONFIG['speed_vars']):
        value = [v1_speed, v2_speed, sur2_speed, sur3_speed][i]
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{value:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_5_test_case_{idx:03d}.py")
            print("Scenario 5 code block:")
            print(scenario_code)
            for line in scenario_code.splitlines():
                if any(k in line for k in ['queue_vehicle1_target_velocity', 'queue_vehicle2_target_velocity', 'sur_vehicle2_target_velocity', 'sur_vehicle3_target_velocity']):
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换间距
    for var_prefix, base_value, var_suffix in SCENARIO_5_CONFIG['distance_vars']:
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{dist:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_5_test_case_{idx:03d}.py")
            print("Scenario 5 code block:")
            print(scenario_code)
            for line in scenario_code.splitlines():
                if 'offset_1' in line:
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换旁车位置
    for i, (var_prefix, base_value, var_suffix) in enumerate(SCENARIO_5_CONFIG['side_vars']):
        value = [side_offset2, side_offset3][i]
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{value:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_5_test_case_{idx:03d}.py")
            print("Scenario 5 code block:")
            print(scenario_code)
            for line in scenario_code.splitlines():
                if 'offset_2' in line or 'offset_3' in line:
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 更新场景代码
    new_content = new_content[:start_index] + scenario_code + new_content[end_index:]

    # 修改 def main()，匹配任意 default 值
    main_default_pattern = r"(argparser\.add_argument\([^)]*default\s*=\s*['\"])[0-6](['\"][^)]*\))"
    new_main_default = r"\g<1>5\g<2>"
    if not re.search(main_default_pattern, new_content):
        print(f"Warning: Could not find default value in main() for scenario_5_test_case_{idx:03d}.py")
        print("Main function:")
        main_start = new_content.find('def main():')
        main_end = new_content.find('def ', main_start + 1) if new_content.find('def ', main_start + 1) != -1 else len(new_content)
        print(new_content[main_start:main_end])
    new_content = re.sub(main_default_pattern, new_main_default, new_content, count=1)

    # 写入新文件
    output_filename = os.path.join(output_dir, f'scenario_5_test_case_{idx:03d}.py')
    with open(output_filename, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Generated {output_filename} with v1_speed={v1_speed:.1f}, v2_speed={v2_speed:.1f}, sur2_speed={sur2_speed:.1f}, sur3_speed={sur3_speed:.1f}, dist={dist:.1f}, offset_2.y={side_offset2:.1f}, offset_3.y={side_offset3:.1f}")

    # 记录参数
    params_data.append({
        'file': f'scenario_5_test_case_{idx:03d}.py',
        'queue_vehicle1_speed_ms': v1_speed,
        'queue_vehicle2_speed_ms': v2_speed,
        'sur_vehicle2_speed_ms': sur2_speed,
        'sur_vehicle3_speed_ms': sur3_speed,
        'offset_1_x': dist,
        'offset_2_y': side_offset2,
        'offset_3_y': side_offset3
    })

# 保存参数到 Excel
df = pd.DataFrame(params_data)
df.to_excel(os.path.join(output_dir, 'scenario_5_params.xlsx'), index=False)
print("Parameters saved to scenario_5_params.xlsx")

print("Scenario 5 processing completed.")