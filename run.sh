#!/bin/sh

export CARLA_ROOT=/opt/carla-simulator
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla

source /opt/carla-ros-bridge/melodic/setup.bash

/opt/carla-simulator/CarlaUE4.sh &


roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
