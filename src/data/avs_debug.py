#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import math
from avs_reporter import AVSReporter

MAX_TIME = 1000
INTERVAL = 0.1
CARS = 5
FOV = 110

def main():
    TIME = 0
    actor_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        master_actor_list = world.get_actors()
        mapp = world.get_map()
        blueprint_library = world.get_blueprint_library()

        vx = None
        for a in master_actor_list:
            if sys.argv[1] in a.type_id:
                vx = a
        if not vx:
            print("{} not found!".format(sys.argv[1]))
        actor_list.append(vx)
        vx_reporter = AVSReporter(TIME,vx,mapp,world.get_actors(),INTERVAL,FOV)

        for i in range(CARS):
            bp = random_vehicle(blueprint_library)
            v_init = random.choice(mapp.get_spawn_points())
            v = quick_actor(bp, v_init, actor_list, world)
            v.set_autopilot(True)

        CRAZY_TIME = 5
        NORMAL_TIME = 5
        dt = 0
        CRAZY = False
        while TIME < MAX_TIME:
            # Can also report other vehicles' affordance vectors
            print(vx_reporter.report(vx,mapp,world.get_actors())[0]) 
            print(vx_reporter.report(vx,mapp,world.get_actors())[1]) 
            print("CRAZY:",CRAZY)
            vx.set_autopilot(True)
           # if CRAZY:
           #     vx.set_autopilot(False)
           #     vx_wp = mapp.get_waypoint(vx.get_location(),project_to_road=True)
           #     if vx_wp.get_left_lane() and str(vx_wp.get_left_lane().lane_type) == 'Driving':
           #         print("Now turn left!")
           #         vx.apply_control(carla.VehicleControl(throttle=0.1, steer=-0.01))
           #     elif vx_wp.get_right_lane() and str(vx_wp.get_right_lane().lane_type) == 'Driving':
           #         print("Now turn right!")
           #         vx.apply_control(carla.VehicleControl(throttle=0.1, steer=0.01))

           #     dt += INTERVAL
           #     if dt >= CRAZY_TIME:
           #         dt = 0
           #         CRAZY = False
           # else:
           #     vx.set_autopilot(True)
           #     dt += INTERVAL
           #     if dt >= NORMAL_TIME:
           #         dt = 0
           #         CRAZY = True


            time.sleep(INTERVAL)
            TIME += INTERVAL

    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')

def random_vehicle(blueprint_library, only_car=True):
    vehicles = blueprint_library.filter('vehicle.*')
    if only_car:
        vehicles = [x for x in vehicles if int(x.get_attribute('number_of_wheels')) >= 4]
    bp = random.choice(vehicles)
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    return bp

def quick_actor(bp,loc,actor_list,world,verbose=True):
    v = world.spawn_actor(bp, loc)
    actor_list.append(v)
    if verbose:
        print("crated {}".format(v.type_id))
    return v

if __name__ == "__main__":
    main()
