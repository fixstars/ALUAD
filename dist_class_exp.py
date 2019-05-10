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

        while TIME < MAX_TIME:
            # Can also report other vehicles' affordance vectors
            print(vx_reporter.report(vx,mapp,world.get_actors())[0]) 
            print(vx_reporter.report(vx,mapp,world.get_actors())[1]) 
            
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
