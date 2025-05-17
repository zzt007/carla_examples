'''
此脚本用于循环读取测试用例数据，扩展数据到约400组（仅对主车纵向速度应用变化因子），并调用batchProcessing.py文件，同时为每个测试用例生成独立的Python文件
'''

import subprocess
import time
import pandas as pd
import os
import random

# 定义要运行的脚本路径
script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\batchProcessing.py'

# 定义要读取的Excel文件路径
excel_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\TestCase_0923.xlsx'

# 定义输出文件夹路径
output_dir = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\generateHardEncodeParamScenario'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)  # 创建输出文件夹（如果不存在）

# 读取Excel文件
START_ROW = 3  # 取决于excel文件中测试数据起始行
COLUMNS_TO_READ = [71, 72, 74, 79, 80, 82, 84]  # 初始时刻车辆状态对应的列
df = pd.read_excel(excel_path, skiprows=START_ROW)
try:
    df = df.iloc[:, COLUMNS_TO_READ]
    print('- now print the df.columns', df.columns.tolist())
    print('- first 5 rows of df:\n', df.head())
    print('- NaN values in df:\n', df.isna().sum())
    if df.isna().any().any():
        print("Warning: DataFrame contains NaN values, dropping rows with NaN")
        df = df.dropna()
        print(f"- After dropping NaN, {len(df)} rows remain")
except KeyError as e:
    print(f"KeyError: {e}")
    print("Available columns in the DataFrame:", df.columns.tolist())
    exit(1)

# 定义速度变化因子（仅对主车纵向速度）
SPEED_FACTORS = [0.9, 1.0, 1.1, 1.2]  # 减少10%、不变、增加10%、增加20%
# 目标数据量
TARGET_NUM_RUNS = 400

# 生成扩展数据
expanded_data = []
for i in range(len(df)):
    row_data = df.iloc[i].tolist()
    if any(pd.isna(x) for x in row_data):
        print(f"Skipping row {i} due to NaN values: {row_data}")
        continue
    ego_lon, ego_lat, ego_heading, sur_lon, sur_lat, sur_heading, rel_distance = row_data
    
    for ego_lon_factor in SPEED_FACTORS:
        new_row = [
            max(0.0, min(ego_lon * ego_lon_factor, 30.0)),  # 主车纵向速度（限制0-30 m/s）
            ego_lat,
            ego_heading,
            sur_lon,
            sur_lat,
            sur_heading,
            rel_distance
        ]
        expanded_data.append(new_row)

# 随机抽样到400组
if len(expanded_data) > TARGET_NUM_RUNS:
    expanded_data = random.sample(expanded_data, TARGET_NUM_RUNS)
elif len(expanded_data) < TARGET_NUM_RUNS:
    print(f"Warning: Only {len(expanded_data)} groups generated, less than target {TARGET_NUM_RUNS}")

# 定义每次运行之间的间隔时间（秒）
INTERVAL = 2

# 读取batchProcessing.py的内容
with open(script_path, 'r', encoding='utf-8') as f:
    batch_processing_content = f.read()

# 循环处理扩展数据
for i, row_data in enumerate(expanded_data):
    # 将数据转换为字符串，以使用命令行传参
    data_str = ' '.join(map(str, row_data))
    print(f"Running test {i+1}/{len(expanded_data)} with data: {data_str}")

    # 生成新的Python文件
    test_case_filename = os.path.join(output_dir, f'Scenario_1_{i+249:03d}.py')
    params = row_data
    # 创建硬编码的参数定义
    hardcoded_params = f"""
EGO_VEHICLE_SPEED_LON = {params[0]}  # 主车初始纵向速度
EGO_VEHICLE_SPEED_LAT = {params[1]}  # 主车初始横向速度
EGO_VEHICLE_HEADING = {params[2]}  # 主车初始航向角
SUR_VEHICLE_SPEED_LON = {params[3]}  # 周车初始纵向速度
SUR_VEHICLE_SPEED_LAT = {params[4]}  # 周车初始横向速度
SUR_VEHICLE_HEADING = {params[5]}  # 周车初始航向角
INITIAL_REL_DISTANCE = {params[6]}  # 主车和周车初始相对间距
print('-------- now print the ego_vehicle_speed_lon : ', EGO_VEHICLE_SPEED_LON)
"""

    # 找到需要替换的参数解析部分
    start_marker = "# set up some const variables for the simulation, which will be updated from the .xlsx file"
    end_marker = "print('-------- now print the ego_vehicle_speed_lon : ',EGO_VEHICLE_SPEED_LON)"
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