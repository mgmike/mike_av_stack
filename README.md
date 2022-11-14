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

Okay I just restarted from scratch and got another error while building carla from source. I can either use master branch and change the Setup.sh script and others from clang8 to clang10 or use dev env
Got error https://github.com/carla-simulator/carla/issues/5886
to fix https://stackoverflow.com/questions/40790943/usr-bin-ld-cannot-find-lstdc-for-ubuntu-while-trying-to-swift-build-perfe,
sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/libstdc++.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/libstdc++.so.6

On master new error:
CMake Error at cmake/modules/CheckCompilerVersion.cmake:72 (message):
  libstdc++ version must be at least 4.8.

  

Trying dev

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

export CARLA_ROOT=/opt/carla-simulator
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla

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

You also will need simple-pid for manual control.

`pip install --user simple-pid`

Initially, the installed directory is not in the path, so I needed to add it to the path.

`export PATH=$PATH:/home/mike/.local/lib/python3.7/site-packages`

## Running

Make sure to source the carla ros bridge enviornment

`source /opt/carla-ros-bridge/melodic/setup.bash`

I am starting by running the basic carla ros bridge with an ego vehicle. You can find more info about this in the [docs.](https://carla.readthedocs.io/en/0.9.9/ros_launchs/#carla_ego_vehiclelaunch)

`roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch`

