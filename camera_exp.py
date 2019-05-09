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

# TO DO: Make all global vars to be argparser cli flags
# Resolution of the image
R = ('280', '210')
# Camera Field of View
FOV = '110'
# Camera location relative to vehicle and the exact location might be differ 
# with different vehicle
LOC = ("1.2","0","0.5")


def main():
    '''
    Some experiments with cameras
    '''

    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle'))

        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, transform)

        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        vehicle.set_autopilot(True)
        box = vehicle.bounding_box

        camera_transform_front = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera_transform_left = carla.Transform(carla.Location(x=1, y=-0.5, z=2.4))
        camera_transform_left.rotation.yaw -= 90
        camera_transform_left.rotation.pitch -= 20
        camera_transform_right = carla.Transform(carla.Location(x=1, y=+0.5, z=2.4))
        camera_transform_right.rotation.yaw += 90
        camera_transform_right.rotation.pitch -= 20
        camera_transform_spect = carla.Transform(carla.Location(x=1, z=10))
        camera_transform_spect.rotation.yaw += 180
        camera_transform_spect.rotation.pitch -= 90
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', R[0])
        camera_bp.set_attribute('image_size_y', R[1])
        camera_bp.set_attribute('fov', FOV)
        camera_front = world.spawn_actor(camera_bp, camera_transform_front, attach_to=vehicle)
        camera_left = world.spawn_actor(camera_bp, camera_transform_left, attach_to=vehicle)
        camera_right = world.spawn_actor(camera_bp, camera_transform_right, attach_to=vehicle)
        camera_spect = world.spawn_actor(camera_bp, camera_transform_spect, attach_to=vehicle)
        actor_list.append(camera_front)
        actor_list.append(camera_left)
        actor_list.append(camera_right)
        actor_list.append(camera_spect)
        print('created %s' % camera_front.type_id)
        print('created %s' % camera_left.type_id)
        print('created %s' % camera_right.type_id)

        camera_front.listen(lambda image: image.save_to_disk('410tmp/front/%06d.png' % image.frame_number))
        camera_left.listen(lambda image: image.save_to_disk('410tmp/left/%06d.png' % image.frame_number))
        camera_right.listen(lambda image: image.save_to_disk('410tmp/right/%06d.png' % image.frame_number))
        camera_spect.listen(lambda image: image.save_to_disk('410tmp/spect/%06d.png' % image.frame_number))

        transform.location += carla.Location(x=40, y=-3.2)
        transform.rotation.yaw = -180.0
        for _ in range(0, 10):
            transform.location.x += 8.0
            bp = random.choice(blueprint_library.filter('vehicle'))
            npc = world.try_spawn_actor(bp, transform)
            if npc is not None:
                actor_list.append(npc)
                npc.set_autopilot()
                print('created %s' % npc.type_id)

        time.sleep(10)

    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')


if __name__ == '__main__':

    main()
