'''
此脚本为 multi_scenario.py 的场景3生成400个硬编码参数的 Python 文件。
队列车辆（queue_vehicle1, queue_vehicle2, queue_vehicle3）使用相同的速度因子和间距因子。
旁车（sur_vehicle）的速度和起始位置分别调整，offset_3.y 非零避免车辆重叠。
总组合数为 7 * 7 * 7 * 4 = 1372，随机抽样400组。
动态检测缩进，确保参数替换每行独立，保留原始 def main()。
'''

import os
import itertools
import random
import re

# 定义输入和输出路径
input_script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\carla_examples\\multi_scenario.py'
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generateHardEncodeParamScenario_3'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 定义因子
SPEED_FACTORS = [0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3]  # 速度因子
DISTANCE_FACTORS = [0.5, 0.6, 0.8, 1.0, 1.1, 1.2, 1.5]  # 间距因子
SIDE_OFFSETS = [-5.0, -3.0, 3.0, 5.0]  # 旁车位置偏移，确保 y 非零
TARGET_NUM_FILES = 400  # 目标生成400个文件

# 场景3参数配置
SCENARIO_3_CONFIG = {
    'queue_speed_vars': [
        ('queue_vehicle1_target_velocity = carla.Vector3D(y=-', '10(\.0)?', ',x=0,z=0)'),
        ('queue_vehicle2_target_velocity = carla.Vector3D(y=-', '10(\.0)?', ',x=0,z=0)'),
        ('queue_vehicle3_target_velocity = carla.Vector3D(y=-', '10(\.0)?', ',x=0,z=0)')
    ],
    'sur_speed_var': ('sur_vehicle_target_velocity = carla.Vector3D(y=-', '15(\.0)?', ',x=0,z=0)'),
    'distance_vars': [
        ('offset_1 = carla.Location(x=', '-20(\.0)?', ',y=0,z=0)'),
        ('offset_2 = carla.Location(x=', '-20(\.0)?', ',y=0,z=0)')
    ],
    'side_var': ('offset_3 = carla.Location(x=0,y=', '5(\.0)?', ',z=0)'),
    'scenario_start': "if args.scenario == '3':",
    'scenario_end': "elif args.scenario == '4':"
}

# 读取 multi_scenario.py 内容
with open(input_script_path, 'r', encoding='utf-8') as f:
    template_content = f.read()

# 检测场景3代码块的缩进
start_marker = SCENARIO_3_CONFIG['scenario_start']
end_marker = SCENARIO_3_CONFIG['scenario_end']
start_index = template_content.find(start_marker)
end_index = template_content.find(end_marker)
if start_index == -1 or end_index == -1:
    print("Error: Could not find scenario 3 code block in multi_scenario.py")
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
queue_combinations = list(itertools.product(SPEED_FACTORS, DISTANCE_FACTORS))  # 7 * 7 = 49
sur_combinations = list(itertools.product(SPEED_FACTORS, SIDE_OFFSETS))  # 7 * 4 = 28
total_combinations = len(queue_combinations) * len(sur_combinations)  # 49 * 28 = 1372
print(f"Total possible combinations for scenario 3: {total_combinations}")

# 随机抽样400组
all_combinations = list(itertools.product(queue_combinations, sur_combinations))
if len(all_combinations) < TARGET_NUM_FILES:
    print(f"Error: Only {len(all_combinations)} combinations available, less than {TARGET_NUM_FILES}")
    exit(1)
selected_combinations = random.sample(all_combinations, TARGET_NUM_FILES)

# 为场景3生成文件
for idx, ((queue_speed_factor, distance_factor), (sur_speed_factor, side_offset)) in enumerate(selected_combinations, 1):
    new_content = template_content
    scenario_code = new_content[start_index:end_index]

    # 计算参数值
    queue_speed = max(5.0, min(10.0 * queue_speed_factor, 20.0))  # 队列车辆速度，5-20 m/s
    print(f"sur_speed_factor: {sur_speed_factor}, queue_speed_factor: {queue_speed_factor}, distance_factor: {distance_factor}")
    sur_speed = max(5.0, min(15.0 * sur_speed_factor, 20.0))  # 旁车速度，5-20 m/s
    distance = -max(10.0, min(abs(-20.0 * distance_factor), 50.0))  # 间距，-10到-50 m

    # 替换队列车辆速度
    for var_prefix, base_value, var_suffix in SCENARIO_3_CONFIG['queue_speed_vars']:
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{queue_speed:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_3_test_case_{idx:03d}.py")
            for line in scenario_code.splitlines():
                if 'queue_vehicle' in line and 'target_velocity' in line:
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换旁车速度
    var_prefix, base_value, var_suffix = SCENARIO_3_CONFIG['sur_speed_var']
    old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
    new_line = f"{indent}{var_prefix}{sur_speed:.1f}{var_suffix}\n"
    if not re.search(old_pattern, scenario_code, re.MULTILINE):
        print(f"Warning: Could not find pattern '{old_pattern}' in scenario_3_test_case_{idx:03d}.py")
        for line in scenario_code.splitlines():
            if 'sur_vehicle_target_velocity' in line:
                print(f"Found line: '{line}'")
    scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换间距
    for var_prefix, base_value, var_suffix in SCENARIO_3_CONFIG['distance_vars']:
        old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
        new_line = f"{indent}{var_prefix}{distance:.1f}{var_suffix}\n"
        if not re.search(old_pattern, scenario_code, re.MULTILINE):
            print(f"Warning: Could not find pattern '{old_pattern}' in scenario_3_test_case_{idx:03d}.py")
            for line in scenario_code.splitlines():
                if 'offset_1' in line or 'offset_2' in line:
                    print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 替换旁车位置
    var_prefix, base_value, var_suffix = SCENARIO_3_CONFIG['side_var']
    old_pattern = rf"^{indent}{re.escape(var_prefix)}{base_value}{re.escape(var_suffix)}(\s*\n)?"
    new_line = f"{indent}{var_prefix}{side_offset:.1f}{var_suffix}\n"
    if not re.search(old_pattern, scenario_code, re.MULTILINE):
        print(f"Warning: Could not find pattern '{old_pattern}' in scenario_3_test_case_{idx:03d}.py")
        for line in scenario_code.splitlines():
            if 'offset_3' in line:
                print(f"Found line: '{line}'")
        scenario_code = re.sub(old_pattern, new_line, scenario_code, count=1, flags=re.MULTILINE)

    # 更新场景代码
    new_content = new_content[:start_index] + scenario_code + new_content[end_index:]

    # 修改 def main()，仅更改 default='6' 为 default='3'
    main_default_pattern = r"(argparser\.add_argument\([^)]*default\s*=\s*['\"])6(['\"][^)]*\))"
    new_main_default = r"\g<1>3\g<2>"
    if not re.search(main_default_pattern, new_content):
        print(f"Warning: Could not find default='6' in main() for scenario_3_test_case_{idx:03d}.py")
    new_content = re.sub(main_default_pattern, new_main_default, new_content, count=1)

    # 写入新文件
    output_filename = os.path.join(output_dir, f'scenario_3_test_case_{idx:03d}.py')
    with open(output_filename, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Generated {output_filename} with queue_speed={queue_speed:.1f}, distance={distance:.1f}, sur_speed={sur_speed:.1f}, offset_3.y={side_offset:.1f}")

print("Scenario 3 processing completed.")