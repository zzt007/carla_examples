import glob
import os
import sys
import time
import random
import time
import numpy as np
import cv2	
# connect to carla
try:
    sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if sys.maxsize > 2**32 else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
 
actor_list =  []

try:
    #create client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    #world connection
    world = client.get_world() 
    #get blueprint libarary
    blueprint_library = world.get_blueprint_library()
    #Choose a vehicle blueprint which name is model3 and select the first one
    bp = blueprint_library.filter("model3")[0]
    print(bp)
    #Returns a list of recommended spawning points and random choice one from it
    spawn_points = world.get_map().get_spawn_points()
    spawn_point1 = random.choice(world.get_map().get_spawn_points())
    spawn_point2 = random.choice(world.get_map().get_spawn_points()) 
    #spawn vehicle to the world by spawn_actor method
    vehicle1 = world.spawn_actor(bp,spawn_point1)
    vehicle2 = world.spawn_actor(bp,spawn_point2)
    #control the vehicle
    vehicle1.set_autopilot(enabled=True)
    vehicle2.set_autopilot(enabled=True)
    # vehicle.apply_control(carla.VehicleControl(throttle=0.1,steer=0.0))
    #add vehicle to the actor list
    actor_list.append(vehicle1)
    actor_list.append(vehicle2)
    time.sleep(20)

finally:
    for actor in actor_list:
        print('destroying %s' % actor.type_id)
        actor.destroy()
print("All cleaned up!")