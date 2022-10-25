# Autonomous car simulator

This project is going to be a big one for me. I plan on implementing, piece by piece, the self driving stack based off of Carla. I will first start with behavior as it is the highest level, and controls all other levels. I will move down the stack piece by piece adding localization and prediciton. Then I will inplement sensor fusion and control.

I am going to use the Carla simulator and ROS to send data across modules. I aim to make this portible and to have each module abstracted such that I can run sensor fusion models on another computer that collects camera information then send the controls back to the computer running the simulation.

This README will be a mess for now until I can get this working

## Requirements

This project requires Ros2 (Foxy Fitzroy) to listen to topics. To generate the simulation, I am using the following on a seperate computer:

Ubuntu 20.04
Unreal engine (4.26) and Carla (0.9.13 from source): https://carla.readthedocs.io/en/latest/build_linux/
Carla ros bridge (0.9.13) and Ros2 (Foxy Fitzroy from source): https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html


## Installation Issues

carla-ros-bridge colcon build failed. Turns out it would somehow find my other ros2 humble installation even though there was no reference to it in PATH or PYTHONPATH. Fine ill delete it. That worked. On to the next

The carla-simulator installed in apt only had a python3.7.egg and building carla-ros-bridge required python3.6.
So I started from scratch, apt remove carla-sim, deleted ~/carla
Re install following very closely https://carla.readthedocs.io/en/latest/build_linux/
On make PythonAPI i get this gem: clang 8 is required, but it's not installed.

Retrying with clang set to 10 but that didn't work.

Turns out I need to edit a few files such as Setup.sh and manually set clang to 10. That works, back to carla-ros-bridge



Build:
Carla -> CMake
Ros2 -> through apt
carla-ros-bridge -> colcon

Ok this is super confusing but here goes

- colcon
    - catkin
    - ament_package 
        - cmake
        - setuptools (python)


# Next steps for tomorrow

find the best way to get ros2 and start subbing to topics from carla-ros-bridge even though they dont exist. Maybe docker? 


## Installation


- Docker
    - Anaconda
    - Python: 3.7
    - Tensorflow:
    - OpenCV:
    - ROS:
    - rospy:
    - catkin: 
    - python3-catkin-pkg-modules
    - python3-rospkg-modules
- Unreal Engine: 
- Carla: 


## Running

First, roscore must always be ran first

