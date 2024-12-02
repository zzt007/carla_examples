import glob
import sys
import carla
import threading
import random
import time
import keyboard
import carla.libcarla
import argparse
import math

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


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def spawn_vehicle_by_point(world,spawn_point:carla.Transform,color:str):
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

def get_spawn_point_by_road_id(map,target_road_id, target_lane_id):
    waypoints = map.generate_waypoints(2.0)
    for waypoint in waypoints:
        if waypoint.road_id == target_road_id:
            lane_id = waypoint.lane_id
            if lane_id == target_lane_id:
                location = waypoint.transform.location
                location.z = 1
                spawn_point = carla.Transform(location, waypoint.transform.rotation)
                break
    return spawn_point

# ues keyboard 'tab' button to change the view perspective 
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

# set a common and fixed spectator 
def set_spectator(world,vehicle):
    world.get_spectator().set_transform(
    carla.Transform(vehicle.get_transform().location +
    carla.Location(z=50),
    carla.Rotation(pitch=-90)))

# the cut in distance between the ego vehicle and its surround vehicle
def should_cut_in(sur_vehicle, ego_vehicle, dis_to_cut) -> bool:
    location1 = sur_vehicle.get_transform().location
    location2 = ego_vehicle.get_transform().location
    rel_x = location2.x - location1.x
    rel_y = location2.y - location1.y
    rel_dis = math.sqrt(rel_x ** 2 + rel_y ** 2)
    if rel_dis <= dis_to_cut: # you can change the dis_to_cut value by yourself
        cut_in_flag = True 
    else:
        cut_in_flag = False
    return cut_in_flag

def speed_control_by_pid(vehicle,pid,target_speed):
    control_signal = pid.run_step(target_speed=target_speed, debug=False)
    throttle = max(min(control_signal, 1.0), 0.0) # you can change the throttle value 
    brake = 0.0
    if control_signal < 0.0:
        brake = abs(control_signal)
        throttle = 0.0
    vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))

def cal_target_route(map,vehicle,lane_change_direction:str,target_dis:float):
    '''
    params:
    vehicle : the vehicle need to change lane
    lane_change_direction : the direction of lane change
    target_dis : creates a list of waypoints within an approximate distance (target_dis)
    '''
    # global planner
    global_planner = GlobalRoutePlanner(map, 2.0)
    # get the lane change vehicle current waypoint , in order to use the waypoint API
    lane_change_vehicle_location = vehicle.get_transform().location
    lane_change_vehicle_waypoint = map.get_waypoint(lane_change_vehicle_location)
    # set up the lane change direction by yourself,and use the WayPoint API to force lane change
    if 'left' in lane_change_direction:
        target_org_waypoint = lane_change_vehicle_waypoint.get_left_lane()
    elif 'right' in lane_change_direction:
        target_org_waypoint = lane_change_vehicle_waypoint.get_right_lane()
        
    # get the next waypoint location
    target_location = target_org_waypoint.next(target_dis)[0].transform.location
    # get the route
    route = global_planner.trace_route(lane_change_vehicle_location, target_location)
    return route
    
    
# the main simulation process 
def simulation(args,world):
    map = world.get_map()

    spawn_points = map.get_spawn_points()

    if args.scenario == '3':
        try:
            IS_USE_SPECIFIC_SPAWN_POINT = False
            if  IS_USE_SPECIFIC_SPAWN_POINT:
                # if use the specific spawn point to set up the queue vehicle1 
                queue_vehicle1_spawn_point = spawn_points[0]
            else:
                # if use the road_id and lane_id to set up the spawn point, this settings use for Town06 map
                queue_vehicle1_spawn_point = get_spawn_point_by_road_id(map,
                                                                        target_road_id=38,
                                                                        target_lane_id=-5)
      
            # spawn the vehicles
            queue_vehicle1 = spawn_vehicle_by_point(world,
                                                    queue_vehicle1_spawn_point,
                                                    '255,255,255')
            
            offset_1 = carla.Location(x=-20,y=0,z=0)
            queue_vehicle2_spawn_point = spawn_sur_vehicle_by_offset(queue_vehicle1_spawn_point,offset_1)
            queue_vehicle2 = spawn_vehicle_by_point(world,
                                                    queue_vehicle2_spawn_point,
                                                    '0,0,0')
            
            # tip: the carla.Location belike a flag, so the change of Location based on last time change
            offset_2 = carla.Location(x=-20,y=0,z=0)
            queue_vehicle3_spawn_point = spawn_sur_vehicle_by_offset(queue_vehicle1_spawn_point,offset_2)
            queue_vehicle3 = spawn_vehicle_by_point(world,
                                                    queue_vehicle3_spawn_point,
                                                    '255,0,0')
            
            set_spectator(world,queue_vehicle2)
            
            # if use the autopilot 
            # queue_vehicle1.set_autopilot(True)
            # queue_vehicle2.set_autopilot(True)
            # queue_vehicle3.set_autopilot(True)
            
            # if use the target velocity, the value can be changed when you need
            queue_vehicle1_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            queue_vehicle2_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            queue_vehicle3_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            
            queue_vehicle1_destination = carla.Transform(carla.Location(queue_vehicle1_spawn_point.location.x + 100,
                                                                        queue_vehicle1_spawn_point.location.y,
                                                                        queue_vehicle1_spawn_point.location.z),
                                                                        queue_vehicle1_spawn_point.rotation)

            IS_ARRIVE_DESTINATION = False
            
            while not IS_ARRIVE_DESTINATION:
                queue_vehicle1.set_target_velocity(queue_vehicle1_target_velocity)
                queue_vehicle2.set_target_velocity(queue_vehicle2_target_velocity)
                queue_vehicle3.set_target_velocity(queue_vehicle3_target_velocity)
                
                # set the destination of the vehicle queue, if vehicle1 arrive the destination, the queue will stop
                
                
                if queue_vehicle1.get_location().distance(queue_vehicle1_destination.location) < 1.0:
                    IS_ARRIVE_DESTINATION = True
                    print('- arrive the destination, end of the simulation! ')
                    
                    # destory the vehicles
                    queue_vehicle1.destroy()
                    queue_vehicle2.destroy()
                    queue_vehicle3.destroy()
                    
        except KeyboardInterrupt:
            queue_vehicle1.destroy()
            queue_vehicle2.destroy()
            queue_vehicle3.destroy()
            print('\nDone.')
        
    elif args.scenario == '4':
        '''
        in the scenarion 4, the queue vehicle1 needs to lane change and follow the other vehicles
        '''
        try:
            IS_USE_SPECIFIC_SPAWN_POINT = False
            if  IS_USE_SPECIFIC_SPAWN_POINT:
                # if use the specific spawn point to set up the queue vehicle1 
                queue_vehicle1_spawn_point = spawn_points[0]
            else:
                # if use the road_id and lane_id to set up the spawn point, this settings use for Town06 map
                queue_vehicle1_spawn_point = get_spawn_point_by_road_id(map,
                                                                        target_road_id=38,
                                                                        target_lane_id=-5)
                
            # spawn the vehicle1
            queue_vehicle1 = spawn_vehicle_by_point(world,
                                                    queue_vehicle1_spawn_point,
                                                    '255,255,255')
            print('- now print the queue_vehicle1 type is : ',type(queue_vehicle1))
            
            # set up the vehicle2 and 3 at the right lane 
            offset_1 = carla.Location(x=20,y=3.5,z=0) # the y set 5.5 is not plausable, fix me!
            queue_vehicle2_spawn_point = spawn_sur_vehicle_by_offset(queue_vehicle1_spawn_point,offset_1)
            queue_vehicle2 = spawn_vehicle_by_point(world,
                                                    queue_vehicle2_spawn_point,
                                                    '0,0,0')
            
            offset_2 = carla.Location(x=20,y=0,z=0) 
            queue_vehicle3_spawn_point = spawn_sur_vehicle_by_offset(queue_vehicle1_spawn_point,offset_2)
            queue_vehicle3 = spawn_vehicle_by_point(world,
                                                    queue_vehicle3_spawn_point,
                                                    '255,0,0')
            
            # let the vehicle2 and 3 to drive
            queue_vehicle2_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            queue_vehicle3_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            
            queue_vehicle2.set_target_velocity(queue_vehicle2_target_velocity)
            queue_vehicle3.set_target_velocity(queue_vehicle3_target_velocity)
                      
            # use pid to control the vehicle1
            args_lateral_dict = {'K_P': 0.8, 'K_D': 0.8, 'K_I': 0.70, 'dt': 1.0 / 10.0}
            args_long_dict = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 10.0}
            PID = VehiclePIDController(queue_vehicle1, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)

            # longitudinal control pid
            vehicle_longitudinal_pid = PIDLongitudinalController(queue_vehicle1, K_P=1.0, K_D=0.1, K_I=0.05)
            waypoints = None
            waypoint_index = 0
            NEED_CAL_ROUTE = True
            CUT_IN_FLAG = False
            IS_ARRIVE_DESTINATION = False
            
            target_speed = 12 * 3.6 # 10 m/s
            set_spectator(world,queue_vehicle2)
            
            while not IS_ARRIVE_DESTINATION:                
                if CUT_IN_FLAG:
                    if NEED_CAL_ROUTE:
                        waypoints = cal_target_route(map,
                                                    queue_vehicle1,
                                                    lane_change_direction='right',
                                                    target_dis=60)
                        
                        NEED_CAL_ROUTE = False
                        
                    if waypoints is not None and waypoint_index < len(waypoints):
                        target_waypoint = waypoints[waypoint_index][0]
                        transform = queue_vehicle1.get_transform()
                        """
                        distance_vehicle():
                        Returns the 2D distance from a waypoint to a vehicle
                            :param waypoint: actual waypoint
                            :param vehicle_transform: transform of the target vehicle
                        """
                        distance2waypoint = distance_vehicle(target_waypoint, transform)
                        # 3.0 is the target_distance_threshold
                        if distance2waypoint < 3.0:
                            waypoint_index += 1
                            if waypoint_index >= len(waypoints):
                                IS_ARRIVE_DESTINATION = True
                                print('- arrive the destination, end of the simulation! ')
                                break
                        else:
                            control = PID.run_step(target_speed, target_waypoint)
                            queue_vehicle1.apply_control(control)
                            
                else:
                    # set up the queue vehicle1 speed
                    speed_control_by_pid(queue_vehicle1,
                                         vehicle_longitudinal_pid,
                                         target_speed)
                    
                    CUT_IN_FLAG = should_cut_in(queue_vehicle1,
                                                queue_vehicle2,
                                                dis_to_cut=20)

        except KeyboardInterrupt:
            queue_vehicle1.destroy()
            queue_vehicle2.destroy()
            queue_vehicle3.destroy()
            print('\nDone.')
            
    elif args.scenario == '5':
        try:
            IS_USE_SPECIFIC_SPAWN_POINT = False
            if  IS_USE_SPECIFIC_SPAWN_POINT:
                # if use the specific spawn point to set up the queue vehicle1 
                queue_vehicle1_spawn_point = spawn_points[0]
            else:
                # if use the road_id and lane_id to set up the spawn point, this settings use for Town06 map
                queue_vehicle1_spawn_point = get_spawn_point_by_road_id(map,
                                                                        target_road_id=37,
                                                                        target_lane_id=-5)

            # spawn the vehicles
            queue_vehicle1 = spawn_vehicle_by_point(world,
                                                    queue_vehicle1_spawn_point,
                                                    '255,255,255')
            
            offset_1 = carla.Location(x=30,y=0,z=0)
            queue_vehicle2_spawn_point = spawn_sur_vehicle_by_offset(queue_vehicle1_spawn_point,offset_1)
            queue_vehicle2 = spawn_vehicle_by_point(world,
                                                    queue_vehicle2_spawn_point,
                                                    '0,0,0')
            
            set_spectator(world,queue_vehicle2)
            
            queue_vehicle2_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            queue_vehicle2.set_target_velocity(queue_vehicle2_target_velocity)
            
            # use pid to control the vehicle1 
            args_lateral_dict = {'K_P': 0.8, 'K_D': 0.8, 'K_I': 0.70, 'dt': 1.0 / 10.0}
            args_long_dict = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.75, 'dt': 1.0 / 10.0}
            PID = VehiclePIDController(queue_vehicle1, args_lateral=args_lateral_dict, args_longitudinal=args_long_dict)

            # longitudinal control pid
            vehicle_longitudinal_pid = PIDLongitudinalController(queue_vehicle1, K_P=1.0, K_D=0.1, K_I=0.05)
            waypoints = None
            waypoint_index = 0
            NEED_CAL_ROUTE = True
            CUT_IN_FLAG = False
            IS_ARRIVE_DESTINATION = False
            
            target_speed = 15 * 3.6 # 15 m/s , set a faster speed for the vehicle1, in order to approach the car ahead before changing lanes
            
            while not IS_ARRIVE_DESTINATION:                
                if CUT_IN_FLAG:
                    if NEED_CAL_ROUTE:
                        waypoints = cal_target_route(map,
                                                    queue_vehicle1,
                                                    lane_change_direction='right',
                                                    target_dis=60)
                        
                        NEED_CAL_ROUTE = False
                        
                    if waypoints is not None and waypoint_index < len(waypoints):
                        target_waypoint = waypoints[waypoint_index][0]
                        transform = queue_vehicle1.get_transform()
                        """
                        distance_vehicle():
                        Returns the 2D distance from a waypoint to a vehicle
                            :param waypoint: actual waypoint
                            :param vehicle_transform: transform of the target vehicle
                        """
                        distance2waypoint = distance_vehicle(target_waypoint, transform)
                        # 3.0 is the target_distance_threshold
                        if distance2waypoint < 3.0:
                            waypoint_index += 1
                            if waypoint_index >= len(waypoints):
                                IS_ARRIVE_DESTINATION = True
                                print('- arrive the destination, end of the simulation! ')
                                break
                        else:
                            control = PID.run_step(target_speed, target_waypoint)
                            queue_vehicle1.apply_control(control)
                else:
                    # set up the queue vehicle1 speed
                    speed_control_by_pid(queue_vehicle1,
                                         vehicle_longitudinal_pid,
                                         target_speed)
                    
                    CUT_IN_FLAG = should_cut_in(queue_vehicle1,
                                                queue_vehicle2,
                                                dis_to_cut=20)

        except KeyboardInterrupt:
            queue_vehicle1.destroy()
            queue_vehicle2.destroy()
            print('\nDone.')
    
    elif args.scenario == '6':
        try:
            IS_USE_SPECIFIC_SPAWN_POINT = False
            if  IS_USE_SPECIFIC_SPAWN_POINT:
                # if use the specific spawn point to set up the queue vehicle1 
                queue_vehicle1_spawn_point = spawn_points[0]
            else:
                # if use the road_id and lane_id to set up the spawn point, this settings use for Town06 map
                queue_vehicle1_spawn_point = get_spawn_point_by_road_id(map,
                                                                        target_road_id=38,
                                                                        target_lane_id=-5)
      
            # spawn the vehicles
            queue_vehicle1 = spawn_vehicle_by_point(world,
                                                    queue_vehicle1_spawn_point,
                                                    '255,255,255')
            
            offset_1 = carla.Location(x=0,y=3.8,z=0)
            queue_vehicle2_spawn_point = spawn_sur_vehicle_by_offset(queue_vehicle1_spawn_point,offset_1)
            queue_vehicle2 = spawn_vehicle_by_point(world,
                                                    queue_vehicle2_spawn_point,
                                                    '0,0,0')
           
            set_spectator(world,queue_vehicle2)
            
            # if use the target velocity, the value can be changed when you need
            queue_vehicle1_target_velocity = carla.Vector3D(x=15,y=0,z=0)
            queue_vehicle2_target_velocity = carla.Vector3D(x=10,y=0,z=0)
            
            queue_vehicle1_destination = carla.Transform(carla.Location(queue_vehicle1_spawn_point.location.x + 100,
                                                                        queue_vehicle1_spawn_point.location.y,
                                                                        queue_vehicle1_spawn_point.location.z),
                                                                        queue_vehicle1_spawn_point.rotation)
            
            # when arrive this speed_limit_destination, the speed will be limited
            speed_limit_destination = carla.Transform(carla.Location(queue_vehicle1_spawn_point.location.x + 30,
                                                            queue_vehicle1_spawn_point.location.y,
                                                            queue_vehicle1_spawn_point.location.z),
                                                            queue_vehicle1_spawn_point.rotation)

            # longitudinal control pid，to slow down the speed under the limit
            vehicle1_longitudinal_pid = PIDLongitudinalController(queue_vehicle1, K_P=1.0, K_D=0.1, K_I=0.05)
            vehicle2_longitudinal_pid = PIDLongitudinalController(queue_vehicle2, K_P=1.0, K_D=0.1, K_I=0.05)
            
            IS_ARRIVE_DESTINATION = False
            IS_ARRIVE_SPEED_LIMIT_DESTINATION = False
            IS_SPEED_LIMIT = False
            
            # use the time to control when the speed will be limited
            time_to_limit_speed = 2 # s            
            time_start = time.time()
            
            while not IS_ARRIVE_DESTINATION:
                # if not IS_SPEED_LIMIT:
                #     queue_vehicle1.set_target_velocity(queue_vehicle1_target_velocity)
                #     queue_vehicle2.set_target_velocity(queue_vehicle2_target_velocity)
                # else:
                #     '''
                #     attention! queue_vehicle_target_velocity is a Vector3D, but def speed_control_by_pid() target_speed is float
                #     so we have to use the Vector3D.x or which axis can in terms of the vehicle longitude direction
                #     '''
                #     speed_control_by_pid(queue_vehicle1,
                #                          vehicle1_longitudinal_pid,
                #                          queue_vehicle1_target_velocity.x * 0.6)   
                    
                #     speed_control_by_pid(queue_vehicle2,
                #                          vehicle2_longitudinal_pid,
                #                          queue_vehicle1_target_velocity.x * 0.6)    
                    
                # if time.time() - time_start >= time_to_limit_speed:
                #     if not IS_SPEED_LIMIT:
                #         IS_SPEED_LIMIT = True
                #         print('- now speed limit! ')
                        
                # use the carla.VehicleControl to control the speed directly           
                if not IS_SPEED_LIMIT:
                    queue_vehicle1.set_target_velocity(queue_vehicle1_target_velocity)
                    queue_vehicle2.set_target_velocity(queue_vehicle2_target_velocity)
                
                    while not IS_ARRIVE_SPEED_LIMIT_DESTINATION:
                        if queue_vehicle1.get_location().distance(speed_limit_destination.location) < 5.0:
                            IS_ARRIVE_SPEED_LIMIT_DESTINATION = True
                            IS_SPEED_LIMIT = True
                            print('- arrive the speed limit destination, start to limit the speed! ')
                else:
                    queue_vehicle1.apply_control(carla.VehicleControl(throttle=0.3,brake=0.5))
                    queue_vehicle2.apply_control(carla.VehicleControl(throttle=0.3,brake=0.4))
                            
                # set the destination of the vehicle queue, if vehicle1 arrive the destination, the queue will stop
                if queue_vehicle1.get_location().distance(queue_vehicle1_destination.location) < 5.0:
                    IS_ARRIVE_DESTINATION = True
                    print('- arrive the destination, end of the simulation! ')
                    
                    # destory the vehicles
                    queue_vehicle1.destroy()
                    queue_vehicle2.destroy()
        
        except KeyboardInterrupt:
            queue_vehicle1.destroy()
            queue_vehicle2.destroy()
            print('\nDone.')
    
    else:
        print('-------------- The scenario is not exist! ---------------')
        exit(0)
    
def main():
    argparser = argparse.ArgumentParser(
        description='Multi Scenario Simulation'
    )
    argparser.add_argument(
        '-s','--scenario',
        default = '4',
        help = 'pick which scenario to run,from 3 to 6',
        type=str
    )
    
    args = argparser.parse_args()
    print('- now Scneario is : ',args.scenario)
    
    # connect to carla
    client = carla.Client(host='127.0.0.1',port = 2000)
    client.set_timeout(10.0)
    IS_USE_CUSTOM_MAP = False
    if IS_USE_CUSTOM_MAP:
        world = client.get_world()
    else:
        world = client.load_world('Town06') 
    
    simulation(args,world)
    
if __name__ == '__main__':
    main()
    

        