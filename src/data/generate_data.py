#!/usr/bin/env python

import os
import sys
import shutil
import glob

try:
    sys.path.append(glob.glob('CARLA_0.9.5/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import queue
import numpy as np
import math
import csv
import argparse
from avs_reporter import AVSReporter


parser = argparse.ArgumentParser(description='Generating simulation data with various configuration')
parser.add_argument('--duration','-du',metavar='int(seconds)', help='Duration of the simulation',type=int,required=True)
parser.add_argument('--name','-n',metavar='str', help='Name of the data', type=str, required=True)
parser.add_argument('--ego-cars','-ec',metavar='int', help='The number of ego vehicles', type=int, default=8)
parser.add_argument('--npc-cars','-nc',metavar='int', help='The number of NPC vehicles to try', type=int, default=1250)
parser.add_argument('--ego-type','-et',metavar='str', help='The type of the ego vehicle', type=str, default='tesla')
#parser.add_argument('--manual','-m',metavar='binary', help='Need one car to manually drive', type=bool, default=False)
parser.add_argument('--fov','-f',metavar='int(degree)', help='Field of View of the dash cam', type=int, default=110)
parser.add_argument('--resolution-x','-rx',metavar='int(pixel)', help='Horizontal resolution of the dash cam', type=int, default=280)
parser.add_argument('--resolution-y','-ry',metavar='int(pixel)', help='Vertical resolution of the dash cam', type=int, default=210)
parser.add_argument('--cam-x','-cx',metavar='float(m)', help='Dash cam location relative to the ego vehicle in x direction', type=float, default=1.2)
parser.add_argument('--cam-y','-cy',metavar='float(m)', help='Dash cam location relative to the ego vehicle in y direction', type=float, default=0)
parser.add_argument('--cam-z','-cz',metavar='float(m)', help='Dash cam location relative to the ego vehicle in z direction', type=float, default=1.5)
parser.add_argument('--cam-yaw','-cry',metavar='float(degree)', help='Dash cam rotation yaw', type=float, default=0)
parser.add_argument('--cam-pitch','-crp',metavar='float(degree)', help='Dash cam rotation pitch', type=float, default=10)
parser.add_argument('--cam-roll','-crr',metavar='float(degree)', help='Dash cam rotation roll', type=float, default=0)
parser.add_argument('--max-dist','-maxd',metavar='float', help='Max distance to the front car to include', type=float, default=80)
# TODO: If debug = false; don't even need to do manual_drive.py
# TODO: execute manual_drive.py inside here and get rid off its cli
parser.add_argument('--debug','-d',metavar='int(0|1|2)', help='Need a debug ego car which can be driven manually(2) or automatically(1)', type=int, default=0)

args = vars(parser.parse_args())

# This intermediate global representation can be removed
MAX_TIME = args['duration']
EGO_CARS = args['ego_cars']
EGO_TYPE = args['ego_type'] 
NPC_CARS = args['npc_cars']
# Camera Field of View
FOV = args['fov']
# Resolution of the image
R = (args['resolution_x'], args['resolution_y'])
# Camera location relative to vehicle and the exact location might be differ 
# with different vehicle
#         (x,y,z) might be better using dicts
CAM_LOC = (args['cam_x'],args['cam_y'],args['cam_z'])
#         (yaw, pitch, roll) 
CAM_ROT = (args['cam_yaw'],args['cam_pitch'],args['cam_roll'])
FPS = 10
INTERVAL = 1/FPS
NAME = "{}".format(args['name'])
CSV_NAME = "{}_labels.csv".format(NAME)
MAXD = args['max_dist']
DEBUG = args['debug']

# TODO: Or just set time limit
# Send images and csv to DGX once exceeds buffer
BUFFER = 5000

class CarNotFoundError(Exception):
    print("Couldn't find the hero debug car")
    
def main():
    TIME = 0
    FRAME_NUMBER = 0
    actor_list = []
    cars = {}
    hero = 0
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        mapp = world.get_map()
        blueprint_library = world.get_blueprint_library()

        settings = world.get_settings()
        settings.synchronous_mode = True
        world.apply_settings(settings)

        csvfile = open(CSV_NAME, 'w', newline='')
        avs_writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)

        # unique vx can be accessed thus driven manually
        if DEBUG:
            hero += 1
            vx = None
            for a in world.get_actors():
                print(a.type_id,EGO_TYPE)
                if EGO_TYPE in a.type_id:
                    vx = a
                    actor_list.append(vx)
            if not vx:
                print("{} not found!".format(EGO_TYPE))
                raise CarNotFoundError 
            if DEBUG == 1:
                vx.set_autopilot(True)

            # actor includes traffic light
            vx_reporter = AVSReporter(TIME,vx,mapp,actor_list,INTERVAL,FOV)
            vx_front_cam = quick_front_cam(blueprint_library,actor_list,world,vx)
            # Need to create separate queues for different vehicles
            image_queuex = queue.Queue()
            vx_front_cam.listen(image_queuex.put)
            
            # Parallel ego-vehicles 
            cars[vx] = (vx_reporter, image_queuex, 'hero')
        # The disadvantabge of using global vars

        for i in range(EGO_CARS-hero):
            v = None
            while not v:
                bp = random_vehicle(blueprint_library,car_type=EGO_TYPE)
                v_init = random.choice(mapp.get_spawn_points())
                v = quick_actor(bp, v_init, actor_list, world)
                if v:
                    v_reporter = AVSReporter(TIME,v,mapp,actor_list,INTERVAL,FOV)
                    lanes = v_reporter.report(v,mapp,actor_list)[1][-1]
                    if lanes < 4:
                        # Might not work since the vehicle is still in the air
                        # Filter out later
                        v = None
                    else:
                        v.set_autopilot(True)
                        image_queue = queue.Queue()
                        v_front_cam = quick_front_cam(blueprint_library,actor_list,world,v)
                        v_front_cam.listen(image_queue.put)
                        cars[v] = (v_reporter, image_queue)
        
        # Create NPC cars
        for i in range(NPC_CARS):
            bp = random_vehicle(blueprint_library)
            v_init = random.choice(mapp.get_spawn_points())
            v = quick_actor(bp, v_init, actor_list, world)
            if v:
                v.set_autopilot(True)

        STARTED = False
        t_start = time.time()
        t_end = t_start + MAX_TIME
        # total qualified frames:
        tqf = 0
        while time.time() < t_end:
            world.tick()
            timestamp = world.wait_for_tick()
            if not STARTED:
                start_frame = timestamp.frame_count
            STARTED = True
            for i,(k,v) in enumerate(cars.items()):
                # avs = list of affordance vectors
                # avss = avs + report
                avss = v[0].report(k,mapp,actor_list)
                avs = avss[1]
                avs = [x if x != None else -1 for x in avs]
                # e.g. v1-frame#
                avs.insert(0,"{}-v{}-{}".format(NAME,i,timestamp.frame_count))
    
                # Filter out entiries that are (not on highway|front cars too far away)
                lanes = avs[-1]
                #dists = avs[5:7] + avs[11:14]
                #dists = [x for x in dists if x<MAXD]

                image = v[1].get()
                if (int(lanes) >= 4):
                    avs_writer.writerow(avs)
                    image.save_to_disk("{}/v{}/{}".format(NAME,i,timestamp.frame_count))
                    csvfile.flush()
                    tqf += 1

                if DEBUG:
                    if len(v) == 3:
                        # Hero Car!
                        print(timestamp)
                        print(avss[0])
                        print(avs)
                else:
                    print("Simulation Time: {} seconds".format(time.time()-t_start))
                    print("Current Frame: {}".format(timestamp.frame_count))
                    tpf = EGO_CARS*(timestamp.frame_count - start_frame)
                    print("Total Possible Frames: {}*{}={}".format(EGO_CARS,timestamp.frame_count-start_frame,tpf))
                    print("Total Actual Frames: {}/{} ({:.3f})".format(tqf, tpf, (tqf+1)/(tpf+1)))



    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False 
        world.apply_settings(settings)
        csvfile.close()
        if not os.path.exists('../../data'):
            os.mkdir('../../data')
        shutil.move("./{}".format(NAME),"../../data")
        shutil.move("./{}".format(CSV_NAME),"../../data/{}".format(NAME))
        print('done.')


def quick_front_cam(blueprint_library,actor_list,world,vx,verbose=True):
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(R[0]))
    cam_bp.set_attribute('image_size_y', str(R[1]))
    cam_bp.set_attribute('fov', str(FOV))
    front_cam_loc = carla.Transform(carla.Location(x=CAM_LOC[0], z=CAM_LOC[2]))
    front_cam_loc.rotation.yaw += CAM_ROT[0] 
    front_cam_loc.rotation.pitch += CAM_ROT[1] 
    front_cam_loc.rotation.roll += CAM_ROT[2]
    front_cam = world.spawn_actor(cam_bp, front_cam_loc, attach_to=vx)
    actor_list.append(front_cam)
    if verbose:
        print("crated {}".format(front_cam.type_id))
    return front_cam

def random_vehicle(blueprint_library, only_car=True, car_type=''):
    vehicles = blueprint_library.filter('vehicle.{}*'.format(car_type))
    if only_car:
        vehicles = [x for x in vehicles if int(x.get_attribute('number_of_wheels')) >= 4]
    bp = random.choice(vehicles)
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    return bp

def quick_actor(bp,loc,actor_list,world,verbose=True):
    v = world.try_spawn_actor(bp, loc)
    if v:
        actor_list.append(v)
        if verbose:
            print("crated {}".format(v.type_id))
    return v

if __name__ == "__main__":
    main()
