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
import keyboard
import pandas as pd
import math
import time 

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

IS_USE_CUSTOM_MAP = False
if IS_USE_CUSTOM_MAP:
    world = client.get_world()
else:
    world = client.load_world('Town05') # 这里使用Town04
    
map = world.get_map()
ego_spectator = world.get_spectator()

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

def updateSpectator(vehicle,world):
    spectator_obj_list = []
    # get the info of vehicle center point 
    bound_x = 0.5 + vehicle.bounding_box.extent.x
    bound_y = 0.5 + vehicle.bounding_box.extent.y
    bound_z = 0.5 + vehicle.bounding_box.extent.z
    
    # get the blueprint of camera
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    
    # set camera AttachmentType
    Atment_SpringArmGhost = carla.libcarla.AttachmentType.SpringArmGhost
    
    Atment_Rigid = carla.libcarla.AttachmentType.Rigid
    
    # set the attachment position of camera
    vehicle_transform_list = [(carla.Location(z=50),
                               carla.Rotation(pitch=-90))]
    
    # set the camera field of view
    camera_transform_list = [
                            (carla.Transform(carla.Location(x=-8, y=0, z=5),
                            carla.Rotation(pitch=15, yaw=0, roll=0)),Atment_SpringArmGhost),

                            (carla.Transform(carla.Location(x=bound_x, y=0, z=bound_z),
                            carla.Rotation(pitch=-15, yaw=180, roll=0)),Atment_SpringArmGhost),

                            (carla.Transform(carla.Location(x=-bound_x, y=0, z=bound_z),
                            carla.Rotation(pitch=-15, yaw=-180, roll=0)),Atment_SpringArmGhost),

                            (carla.Transform(carla.Location(x=bound_x-0.5, y=-bound_y, z=bound_z),
                            carla.Rotation(pitch=-15, yaw=120, roll=20)),Atment_SpringArmGhost),
                            
                            (carla.Transform(carla.Location(x=bound_x-0.5, y=bound_y, z=bound_z),
                            carla.Rotation(pitch=-15, yaw=-120, roll=-20)),Atment_SpringArmGhost)]
    
    # concat two tranform_list 
    spectator_transform_list = vehicle_transform_list + camera_transform_list
    
    for spectator_transform_index in spectator_transform_list:
        # index = 0 为上帝视角，camera无法设置上帝视角（会发生抖动）
        if spectator_transform_list.index(spectator_transform_index) == 0:
            spectator_obj_list.append(spectator_transform_index)
        #spectator_transform_list其余元素为Camera安装参数，下面生成Camera对象
        else:
            camera = world.spawn_actor(camera_bp, spectator_transform_index[0],
            attach_to=vehicle,attachment_type=spectator_transform_index[1])
            spectator_obj_list.append(camera)

    spectator_obj = vehicle_transform_list[0]

    while True:
        # 按Tab键切换视角
        if keyboard.is_pressed('tab'):
            # 上一个spectator的索引
            last_spectator_obj_index = spectator_obj_list.index(spectator_obj)
            # 计算下一个spectator的索引，如果列表索引超限则重新以第0个
            spectator_obj_index = last_spectator_obj_index + 1 if len(spectator_obj_list) - last_spectator_obj_index - 1 > 0 else 0
            spectator_obj = spectator_obj_list[spectator_obj_index]
            time.sleep(0.2)
        
        # 更新视图
        if spectator_obj_list.index(spectator_obj) == 0:
            #设置上帝视图
            vehicle_transform = carla.Transform(vehicle.get_transform().location + spectator_obj_list[0][0],
            spectator_obj_list[0][1])
            world.get_spectator().set_transform(vehicle_transform)
        else:
            #设置其他Camera视图
            world.get_spectator().set_transform(spectator_obj.get_transform())

def calSurVehicleOffset():
    '''
    用于计算两周车相对于出生点、以及两周车之间的偏移量
    '''
    pass

def odometer():
    '''
    用于计算车辆行驶距离
    '''
    pass


try:
    # store the global variables, which should be extracted from the testCase .Excel file
    EGO_VEHICLE_SPEED_LON = 3.50
    EGO_VEHICLE_SPEED_LAT = 0.09
    
    SUR1_VEHICLE_SPEED_LON = 2.50 # SUR1 ： left-proceeding vehicle
    SUR1_VEHICLE_SPEED_LAT = 0.13
    SUR1_X = 432.64
    SUR1_Y = 87.69
    SUR1_YAW = 0
    
    SUR2_VEHICLE_SPEED_LON = 4.50 # SUR2 ： left-rear vehicle
    SUR2_VEHICLE_SPEED_LAT = 0.05
    SUR2_X = 484.71
    SUR2_Y = 87.69
    SUR2_YAW = 0
    
    IS_ARRIVE_DESTINATION = False # use final location replace it, fix me!
    IS_USE_SPECTATOR = False
    
    # 命令行传入的参数
    EGO_VEHICLE_X = params[0]
    EGO_VEHICLE_Y = params[1]
    
    SUR1_VEHICLE_X = params[2]
    SUR1_VEHICLE_Y = params[3]
    
    SUR2_VEHICLE_X = params[4]
    SUR2_VEHICLE_Y = params[5]
    
    
    
    # set up the ego vehicle's initial waypoint to spawn the vehicle
    spawn_points = map.get_spawn_points()
    ego_spawn_point = spawn_points[0]
    
    sur1_spawn_point = spawn_points[1] 
    sur1_spawn_point = spawn_sur_vehicle_by_offset(sur1_spawn_point, carla.Location(x=SUR1_VEHICLE_X, 
                                                                                        y=SUR1_VEHICLE_Y, 
                                                                                        z=0))
    # spawn the ego vehicle and sur vehicle 
    ego_vehicle = spawn_vehicle_by_point(ego_spawn_point,'255,255,255') # 白色是主车
    sur1_vehicle = spawn_vehicle_by_point(sur1_spawn_point,'0,0,0') 


    sur2_offset = carla.Location(x=SUR2_VEHICLE_X, 
                                 y=SUR2_VEHICLE_Y, 
                                 z=0)
    sur2_spawn_point = spawn_sur_vehicle_by_offset(sur1_spawn_point, sur2_offset)
    sur2_vehicle = spawn_vehicle_by_point(sur2_spawn_point,'255,0,0')
    

    # 设置目的地，由于周车直行，方便设置
    DESTINATION = carla.Transform(carla.Location(sur1_spawn_point.location.x + 50, 
                                 sur1_spawn_point.location.y, 
                                 sur1_spawn_point.location.z),sur1_spawn_point.rotation)
    time_start = time.time()
    while not IS_ARRIVE_DESTINATION:

        if IS_USE_SPECTATOR:
            updateSpectator(ego_vehicle,world)
    
        ego_vehicle_speed = carla.Vector3D(x=EGO_VEHICLE_SPEED_LON,
                                        y=EGO_VEHICLE_SPEED_LAT,
                                        z=0)
        
        sur1_vehicle_speed = carla.Vector3D(x=SUR1_VEHICLE_SPEED_LON,
                                           y=SUR1_VEHICLE_SPEED_LAT,
                                           z=0)
        
        sur2_vehicle_speed = carla.Vector3D(x=SUR2_VEHICLE_SPEED_LON,
                                    y=SUR2_VEHICLE_SPEED_LAT,
                                    z=0)

        ego_vehicle.set_autopilot(True)
        # ego_vehicle.enable_constant_velocity(ego_vehicle_speed)
        # print('- now print the ego_vehicle speed : ',ego_vehicle.get_velocity())
        # sur1_vehicle.set_target_velocity(sur1_vehicle_speed)
        sur1_vehicle.set_autopilot(True)
        sur2_vehicle.set_autopilot(True)
        
        # use distance to judge whether arrive the destination
        # if sur1_vehicle.get_location().distance(DESTINATION.location) < 3.0:
        #     IS_ARRIVE_DESTINATION = True
        #     print('- arrive the destination,end of the simulation')
            
        # use time to judge whether arrive the destination
        if time.time() - time_start > 5:
            IS_ARRIVE_DESTINATION = True
            print('- arrive the destination,end of the simulation')
            
    # ego_vehicle.destroy()
    # sur1_vehicle.destroy()
    # sur2_vehicle.destroy()

    
except KeyboardInterrupt:
    ego_vehicle.destroy()
    sur1_vehicle.destroy()
    sur2_vehicle.destroy()
    print('\nDone.')
    


