#!/usr/bin/env python

import argparse
import random
import sys
import time
from matplotlib.hatch import VerticalHatch
import rospy
from std_msgs.msg import String

import carla
import pygame

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from tf.transformations import quaternion_from_euler

class World(object):
    def __init__(self) -> None:
        pass


def carla(args):
    pygame.init()
    world = None
    client = carla.Client(args.host, args.port)
    client.load_world("Town03")
    time.sleep(1)

    display = pygame.display.set_mode(
        (args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF)

    world = client.get_world()
    map = world.get_map()

    # Get a random blueprint.
    #blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
    blueprint = world.get_blueprint_library().filter('vehicle.*')[12]
    #cars = self.world.get_blueprint_library().filter(self._actor_filter)
    #for car in cars:
    #    print(car)
    blueprint.set_attribute('role_name', self.actor_role_name)
    if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        #print(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
    if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)
    if blueprint.has_attribute('is_invincible'):
        blueprint.set_attribute('is_invincible', 'true')

    player = None
    # Spawn the player.
    # if player is not None:
    #     spawn_point = player.get_transform()
    #     spawn_point.location.z += 2.0
    #     spawn_point.rotation.roll = 0.0
    #     spawn_point.rotation.pitch = 0.0
    #     destroy()
    #     player = self.world.try_spawn_actor(blueprint, spawn_point)
    while player is None:
        if not map.get_spawn_points():
            print('There are no spawn points available in your map/town.')
            print('Please add some Vehicle Spawn Point to your UE4 scene.')
            sys.exit(1)
        spawn_point = map.get_spawn_points()[1]
        #print(len(spawn_points))
        #spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        player = world.try_spawn_actor(blueprint, spawn_point)

        '''
        self.actor = self.world.try_spawn_actor(blueprint, actor_spawn)
        '''

        
        pub = rospy.Publisher('chatter', Pose, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = Pose()
            t = player.get_transform()
            pose.position.x =  t.location.x
            pose.position.y =  t.location.y
            pose.position.z =  t.location.z
            q = quaternion_from_euler(t.rotation.roll, t.rotation.pitch, t.rotation.yaw)
            pose.orientation = Quaternion(*q)
            pub.publish(pose)
            rate.sleep()
    

def talker(type, msg):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        carla(args)
        talker()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()