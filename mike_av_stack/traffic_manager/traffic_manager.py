#!/usr/bin/env python3

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse

class TrafficManager:
    def __init__(self, client):
        self.client = client
        self.tm = client.get_trafficmanager(client.port)

    
    #pip3 install -Iv setuptools==47.3.1
    #52.0.0.post20210125
if __name__ == '__main__':
    print(carla.__file__)

    argparser = argparse.ArgumentParser(
        description=__doc__)
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
    args = argparser.parse_args()
    try:
        client = carla.Client(args.host, args.port)
        tm = TrafficManager(client=client)
    except Exception as error:
        print(error)