'''
此脚本用于循环读取测试用例数据，并调用batchProcessing.py文件
'''

import subprocess
import time 
import pandas as pd

# 定义要运行的脚本路径
script_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\batchProcessing.py'

# 定义要读取的Excel文件路径
excel_path = 'D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\examples\\TestCase_0923.xlsx' 

# 读取Excel文件
START_ROW = 3 # 取决于excel文件中测试数据起始行
# COLUMNS_TO_READ = ['BT','BU','BW','CB','CC','CE','CG'] # 初始时刻车辆状态对应的列，此处包括：主车纵向速度、主车横向速度、主车航向角、周车纵向速度、周车横向速度、周车航向角、初始车辆相对间距，有需要可以添加
COLUMNS_TO_READ = [71,72,74,79,80,82,84]
df = pd.read_excel(excel_path,skiprows=START_ROW) # 忽略前START_ROW行,从START_ROW+1 行开始
try:
    df = df.iloc[:,COLUMNS_TO_READ]
    print('-now print the df.columns',df.columns.tolist())
except KeyError as e:
    print(f"KeyError: {e}")
    print(print("Available columns in the DataFrame:", df.columns.tolist()))
    exit(1)
    
# 根据excel文件定义运行次数
# NUM_RUNS = len(df) - 4
NUM_RUNS = 111 # 手动设置次数，这里是所有有效测试行数减去开头四行 ，在0923中，是116- 4 -1 =111,减1是因为索引从0开始

# 定义每次运行之间的间隔时间（秒）
INTERVAL = 10

for i in range(NUM_RUNS):
    # 读取指定列数据

    row_data = df.iloc[i].tolist()
    # 将数据转换为字符串，以使用命令行传参
    data_str = ' '.join(map(str, row_data))
    print(f"Running test {i+1}/{NUM_RUNS} with data: {data_str}")
    
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