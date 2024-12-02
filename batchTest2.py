'''
此脚本用于循环读取测试用例数据，并调用batchProcessing.py文件
'''

import subprocess
import time 
import pandas as pd
import numpy as np

# 定义要运行的脚本路径
script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\batchProcessing2.py'
# 定义每次运行之间的间隔时间（秒）
INTERVAL = 2

# 手动定义三车的offset,使用均匀分布来生成测试用例；依据自定义地图坐标系

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
for x in np.arange(vehicle1_offset_x_min, vehicle1_offset_x_max+1, 1):
    for y in np.arange(vehicle1_offset_y_min, vehicle1_offset_y_max+1, 1):
        vehicle1_offsets.append([x, y])

    
vehicle2_offsets = []
for x in np.arange(vehicle2_offset_x_min, vehicle2_offset_x_max + 1, 1):
    for y in np.arange(vehicle2_offset_y_min, vehicle2_offset_y_max + 1, 1):
        vehicle2_offsets.append((x, y))
        
vehicle3_offsets = vehicle1_offsets

for i in range(min(len(vehicle1_offsets),len(vehicle2_offsets),len(vehicle3_offsets))):
    # 读取指定列数据
    # 将数据转换为字符串，以使用命令行传参
    vehicle1_offset = vehicle1_offsets[i]   
    vehicle2_offset = vehicle2_offsets[i]
    vehicle3_offset = vehicle3_offsets[i]
    
    # 确保两者不一样
    while vehicle2_offset == vehicle3_offset:
        vehicle3_offset = vehicle3_offsets[np.random.randint(0,len(vehicle3_offsets))]
    
    # 将数据转换为字符串，以使用命令行传参
    # 将数据转换为字符串，以使用命令行传参
    data_str = f"{vehicle1_offset[0]} {vehicle1_offset[1]} {vehicle2_offset[0]} {vehicle2_offset[1]} {vehicle3_offset[0]} {vehicle3_offset[1]}"
    print(f"Running test {i+1}/{min(len(vehicle1_offsets), len(vehicle2_offsets), len(vehicle3_offsets))} with data: {data_str}")

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