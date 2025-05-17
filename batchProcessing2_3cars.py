import glob
import sys
import carla
import pandas as pd
import math
import time 
from carla import VehicleControl

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

client = carla.Client(host='127.0.0.1',port = 2000)
client.set_timeout(10.0)

IS_USE_CUSTOM_MAP = True
if IS_USE_CUSTOM_MAP:
    world = client.get_world()
else:
    world = client.load_world('Town05') 
    
map = world.get_map()
actor_list =  []

# spawn the specified vehicle
def spawn_vehicle_by_point(spawn_point:carla.Transform,color:str):
    blueprint_lib = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    blueprint_lib.set_attribute('color', color) 
    vehicle = world.try_spawn_actor(blueprint_lib, spawn_point)
    print('- spawn vehicle successfully! ')
    return vehicle

def spawn_sur_vehicle_by_offset(spawn_point:carla.Transform, offset:carla.Location) -> carla.Transform:
    location = spawn_point.location
    rotation = spawn_point.rotation
    location.x += offset.x
    location.y += offset.y
    location.z += offset.z
    sur_transform = carla.Transform(location, rotation)
    return sur_transform


try:
    # store the global variables, which should be extracted from the testCase .Excel file
    EGO_VEHICLE_SPEED_LON = 3.50
    EGO_VEHICLE_SPEED_LAT = 0.09
    
    SUR1_VEHICLE_SPEED_LON = 2.50 # SUR1 ： left-proceeding vehicle
    SUR1_VEHICLE_SPEED_LAT = 0.13
    
    
    IS_ARRIVE_DESTINATION = False # use final location replace it, fix me!
    IS_USE_SPECTATOR = False
    
    # 命令行传入的参数
    SUR1_VEHICLE_X = params[0]
    SUR1_VEHICLE_Y = params[1]
    SUR2_VEHICLE_SPEED_LON = params[2]
    
    # set up the ego vehicle's initial waypoint to spawn the vehicle
    spawn_points = map.get_spawn_points()
    ego_spawn_point = spawn_points[0]
    sur1_spawn_point = spawn_points[1] 
    sur2_spawn_point = spawn_points[2] 
    
    # 在一个固定出生点附近增加offset，以达到生成多组参数的目的，
    sur1_spawn_point = spawn_sur_vehicle_by_offset(sur1_spawn_point, carla.Location(x=SUR1_VEHICLE_X, 
                                                                                        y=SUR1_VEHICLE_Y, 
                                                                                        z=0))
    # spawn the ego vehicle and sur vehicle 
    ego_vehicle = spawn_vehicle_by_point(ego_spawn_point,'255,255,255') # 白色是主车
    
    
    sur1_vehicle = spawn_vehicle_by_point(sur1_spawn_point,'0,0,0')  # 黑色周车，如果直接用出生点，就不用spawn_sur_vehicle_by_offset函数了；否则解开注释

    

    sur2_vehicle = spawn_vehicle_by_point(sur2_spawn_point, '0,0,255') # 蓝色周车2 ，只改变纵向车速
    
    ego_vehicle_target_speed = carla.Vector3D(x=EGO_VEHICLE_SPEED_LON,
                                    y=EGO_VEHICLE_SPEED_LAT,
                                    z=0)
    
    sur2_vehicle_velocity = carla.Vector3D(x=SUR2_VEHICLE_SPEED_LON,
                                    y=0,
                                    z=0) 

    ego_vehicle.set_autopilot(True)
    sur1_vehicle.set_autopilot(True)
    sur2_vehicle.set_target_velocity(sur2_vehicle_velocity)

    time.sleep(20)
    actor_list.append(ego_vehicle)
    actor_list.append(sur1_vehicle)
    actor_list.append(sur2_vehicle)

finally:
    for actor in actor_list:
        print('destroying %s' % actor.type_id)
        actor.destroy()
    


