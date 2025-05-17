'''
此脚本用于循环读取测试用例数据，并调用batchProcessing2.py文件，同时为每个测试用例生成硬编码参数的Python文件
'''

import subprocess
import time
import pandas as pd
import numpy as np
import os

# 定义要运行的脚本路径
script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\batchProcessing2.py'

# 定义输出文件夹路径
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generateHardEncodeParamScenario_2'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)  # 创建输出文件夹（如果不存在）

# 定义每次运行之间的间隔时间（秒）
INTERVAL = 3

# 手动定义三车的offset，使用均匀分布来生成测试用例；依据自定义地图坐标系

# vehicle1 代表主车
vehicle1_offset_x_min, vehicle1_offset_x_max = -7.0, 7.0
vehicle1_offset_y_min, vehicle1_offset_y_max = -3.5, 3.5

# vehicle2 代表周车
vehicle2_offset_x_min, vehicle2_offset_x_max = -4.0, 8.0
vehicle2_offset_y_min, vehicle2_offset_y_max = -13.0, 14.0

# 生成若干组(x,y)偏移量
num_samples = 50

# 生成vehicle1的偏移量
vehicle1_offsets = []
for x in np.arange(vehicle1_offset_x_min, vehicle1_offset_x_max + 1, 1):
    for y in np.arange(vehicle1_offset_y_min, vehicle1_offset_y_max + 1, 1):
        vehicle1_offsets.append([x, y])

# 生成vehicle2的偏移量
vehicle2_offsets = []
for x in np.arange(vehicle2_offset_x_min, vehicle2_offset_x_max + 1, 1):
    for y in np.arange(vehicle2_offset_y_min, vehicle2_offset_y_max + 1, 1):
        vehicle2_offsets.append([x, y])


# 读取batchProcessing2.py的内容
with open(script_path, 'r', encoding='utf-8') as f:
    batch_processing_content = f.read()

# 循环处理测试用例
num_tests = min(len(vehicle1_offsets), len(vehicle2_offsets))
for i in range(num_tests):
    # 读取指定偏移量
    vehicle1_offset = vehicle1_offsets[i]
    vehicle2_offset = vehicle2_offsets[i]


    # 将数据转换为字符串，以使用命令行传参
    data_str = f"{vehicle1_offset[0]} {vehicle1_offset[1]} {vehicle2_offset[0]} {vehicle2_offset[1]}"
    print(f"Running test {i+1}/{num_tests} with data: {data_str}")

    # 生成新的Python文件
    test_case_filename = os.path.join(output_dir, f'Scenario_2_{i+1:03d}.py')
    params = list(map(float, data_str.split()))
    # 创建硬编码的参数定义
    hardcoded_params = f"""
    EGO_VEHICLE_X = {params[0]}  # 主车X偏移量
    EGO_VEHICLE_Y = {params[1]}  # 主车Y偏移量
    SUR1_VEHICLE_X = {params[2]}  # 周车1 X偏移量
    SUR1_VEHICLE_Y = {params[3]}  # 周车1 Y偏移量
    """

    # 找到需要替换的参数解析部分
    start_marker = "# 命令行传入的参数"
    end_marker = "    SUR1_VEHICLE_Y = params[3]"
    start_index = batch_processing_content.find(start_marker)
    end_index = batch_processing_content.find(end_marker) + len(end_marker)
    if start_index == -1 or end_index == -1:
        print(f"Error: Could not find parameter parsing section in {script_path}")
        exit(1)

    # 替换参数解析部分
    new_content = (
        batch_processing_content[:start_index] +
        hardcoded_params +
        batch_processing_content[end_index:]
    )

    # 删除原有的参数接收逻辑
    param_parsing_start = "# 接收来自自动化测试脚本batchTest.py输出的参数"
    param_parsing_end = "    sys.exit(0)"
    param_parsing_start_index = new_content.find(param_parsing_start)
    param_parsing_end_index = new_content.find(param_parsing_end) + len(param_parsing_end)
    if param_parsing_start_index != -1 and param_parsing_end_index != -1:
        new_content = (
            new_content[:param_parsing_start_index] +
            new_content[param_parsing_end_index:]
        )

    # 写入新文件
    with open(test_case_filename, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f"Generated {test_case_filename}")

    # 调用仿真
    result = subprocess.run(['python', script_path, data_str],
                           capture_output=True,
                           text=True)

    print(result.stdout)
    if result.stderr:
        print(f"Error: {result.stderr}")

    # 等待一段时间
    time.sleep(INTERVAL)

print('- All tests completed.')