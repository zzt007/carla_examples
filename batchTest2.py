'''
此脚本用于循环读取测试用例数据，并调用batchProcessing.py文件
'''

import subprocess
import time 
import pandas as pd
import numpy as np

# 定义要运行的脚本路径
script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\carla_examples\\batchProcessing2_3cars.py'
# 定义每次运行之间的间隔时间（秒）
INTERVAL = 2

# sur_vehicle1 代表周车1, 选定一个固定出生点，将该偏移作用于该出生点
vehicle1_offset_x_min, vehicle1_offset_x_max = -4.0, 8.0
vehicle1_offset_y_min, vehicle1_offset_y_max = -13.0, 14.0

# sur_vehicle2 代表周车2, 使用固定出生点生成， 只改变周车2的车速，间隔1
vehicle2_velocity_x_min, vehicle2_velocity_x_max = 5.0, 16.0 

# 生成sur_vehicle1的offset
sur_vehicle1_offsets = []    
for x in np.arange(vehicle1_offset_x_min, vehicle1_offset_x_max+1, 1):
    for y in np.arange(vehicle1_offset_y_min, vehicle1_offset_y_max+1, 1):
        sur_vehicle1_offsets.append([x, y])

# 生成sur_vehicle2的速度可取值,只在沿车道直行方向进行
sur_vehicle2_velocity_optional = []
for x in np.arange(vehicle2_velocity_x_min, vehicle2_velocity_x_max+1, 1):
    sur_vehicle2_velocity_optional.append([x])

for i in range(len(sur_vehicle1_offsets)):
    # 读取指定列数据
    # 将数据转换为字符串，以使用命令行传参
    sur_vehicle1_offset = sur_vehicle1_offsets[i]   
    sur_vehicle2_velocity = sur_vehicle2_velocity_optional[i]
    
    # 将数据转换为字符串，以使用命令行传参
    # 将数据转换为字符串，以使用命令行传参
    data_str = f"{sur_vehicle1_offset[0]} {sur_vehicle1_offset[1]} {sur_vehicle2_velocity[0]}"
    print(f"Running test {i+1}/{len(sur_vehicle1_offsets)} with data: {data_str}")

    # 调用仿真
    result = subprocess.run(['python',script_path,data_str],
                            capture_output=True,
                            text=True)
    
    print(result.stdout)
    if result.stderr:
        print(f"Error: {result.stderr}")
        
    # 等待一段时间
    time.sleep(INTERVAL)
    
print('- All tests completed.')