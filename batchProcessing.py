'''
This script is used to batch process the test cases, which stored in the .xlsx file.
TODO:
1. 准确读取csv文件中的每个测试用例相关数据
2. 实现在指定起始点位置生成具有指定车速的测试车辆
3. 导航到指定终点位置（目前暂未有指定位置，fix me!）
4. 突出显示导航路径和测试车辆实际换道路径
5. 录制视频demo
'''

import glob
import sys
import carla

import pandas as pd
import math
# 接收来自自动化测试脚本batchTest.py输出的参数（从excel中读取的数据，赋值到该仿真中使用）
if len(sys.argv) > 1:
    data_str = sys.argv[1]
    params = list(map(float, data_str.split()))
else:
    print('- lost the parameters, exit!')
    sys.exit(0)
    
# connect to carla
try:
    sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if sys.maxsize > 2**32 else 'linux-x86_64'))[0])
except IndexError:
    pass

# add the path to the carla module
sys.path.append("D:\\Carla\\CARLA_0.9.15\\WindowsNoEditor\\PythonAPI\\carla\\")

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.controller import VehiclePIDController,PIDLongitudinalController
from agents.tools.misc import draw_waypoints, distance_vehicle, vector, is_within_distance, get_speed
from agents.navigation.behavior_agent import BehaviorAgent

client = carla.Client(host='127.0.0.1',port = 2000)
client.set_timeout(10.0)

world = client.get_world() # 默认使用地图10，修改为自定义地图即可
map = world.get_map()
ego_spectator = world.get_spectator()

# set up some const variables for the simulation, which will be updated from the .xlsx file
EGO_VEHICLE_SPEED_LON = params[0] # 主车初始纵向速度
EGO_VEHICLE_SPEED_LAT = params[1] # 主车初始横向速度
EGO_VEHICLE_HEADING = params[2] # 主车初始航向角
SUR_VEHICLE_SPEED_LON = params[3] # 周车初始纵向速度
SUR_VEHICLE_SPEED_LAT = params[4] # 周车初始横向速度
SUR_VEHICLE_HEADING = params[5] # 周车初始航向角
INITIAL_REL_DISTANCE = params[6] # 主车和周车初始相对间距
print('-------- now print the ego_vehicle_speed_lon : ',EGO_VEHICLE_SPEED_LON)
# spawn the specified vehicle
def spawn_vehicle_by_point(spawn_point,color:str):
    blueprint_lib = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    blueprint_lib.set_attribute('color', color) 
    vehicle = world.try_spawn_actor(blueprint_lib, spawn_point)
    print('- spawn vehicle successfully! ')
    return vehicle

# spawn the sur vehicle by offset
def spawn_sur_vehicle_by_offset(ego_spawn_point, rel_distance) -> carla.Transform:
    '''
    目前通过初始车辆相对间距，假设周车在自车的右邻车道中心，计算出周车的初始位置
    '''
    location = ego_spawn_point.location
    rotation = ego_spawn_point.rotation

    offset_y = 3.5
    location.y += offset_y # 假设车道宽度为3.5m
    offset_x = math.sqrt(rel_distance * rel_distance - offset_y * offset_y)
    location.x += offset_x
    offset_z = 0.0
    location.z += offset_z
    
    sur_transform = carla.Transform(location,rotation)
    return sur_transform

try:
    # 使用地图的spawn_points来作为初始位置的候选
    spawn_points = map.get_spawn_points()
    is_use_custom_map = False
    if not is_use_custom_map:
        ego_spawn_point = spawn_points[0] # 可自行选择在哪，此处选择0号点
    else:
        ego_spawn_point = spawn_points[461] # 选择自定义地图中的    
    
    ego_vehicle = spawn_vehicle_by_point(ego_spawn_point, '255,255,255') # 白色是主车
    
    sur_spawn_point = spawn_sur_vehicle_by_offset(ego_spawn_point, INITIAL_REL_DISTANCE)
    sur_vehicle = spawn_vehicle_by_point(sur_spawn_point, '0,0,0')
    
    IS_ARRIVE_DESTINATION = False # use final location replace it, fix me!
    DESTINATION = carla.Transform(carla.Location(ego_spawn_point.location.x + 50, 
                                 ego_spawn_point.location.y, 
                                 ego_spawn_point.location.z),ego_spawn_point.rotation)
    
    while not IS_ARRIVE_DESTINATION:
        ego_vehicle_speed = carla.Vector3D(x=EGO_VEHICLE_SPEED_LON,
                                           y=EGO_VEHICLE_SPEED_LAT,
                                           z=0)
        
        sur_vehicle_speed = carla.Vector3D(x=SUR_VEHICLE_SPEED_LON,
                                           y=SUR_VEHICLE_SPEED_LAT,
                                           z=0)
        # 自车规划路径,均是通过地图上的waypoints来计算
        ego_agent = BehaviorAgent(ego_vehicle, behavior='normal')
        ego_agent.set_destination(ego_vehicle.get_location(), DESTINATION.location)

        # 单纯使用waypoints来检测自车是否达到目的地
        
        
        ego_vehicle.set_target_velocity(ego_vehicle_speed)
        sur_vehicle.set_target_velocity(sur_vehicle_speed)
        
        
        if ego_vehicle.get_location().distance(DESTINATION.location) < 5.0:
            IS_ARRIVE_DESTINATION = True
            print('- arrive the destination,end of the simulation')
            
    ego_vehicle.destroy()
    sur_vehicle.destroy()
    
except KeyboardInterrupt:
    ego_vehicle.destroy()
    sur_vehicle.destroy()
    print('\nDone.')
    





    





