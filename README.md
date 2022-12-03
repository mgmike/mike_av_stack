# Autonomous car simulator

This project is going to be a big one for me. I plan on implementing, piece by piece, the self driving stack based off Carla. I will first start with the lowest levels then add on higher levels of the stack. For instance, camera and lidar object detection will be first, and behavior and control will be last.

I am going to use the Carla simulator bult on Unreal Engine to run simulations and ROS to send data across modules. I aim to make this portible and to have each module abstracted such that I can run my code indipendant of Carla and open the posibility of running the simulations on a completely seperate machine. This enables me to run a simulation on computer A, send data over ros to computer B which is running this code. In another hypothetical scenario, raspberry pi A collects camera and lidar data from real sensors attached to a vehcile and sends data via ros to computer B in the vehicle running this project.

# Requirements

OS: Ubuntu 18.04\
Unreal engine: 4.26\
carla-simulator/bionic (apt installation): 0.9.13\
Ros (apt installation): Melodic\
carla-ros-bridge/bionic (apt installation): 0.9.10-1

<!-- 
This is for when I eventually create a fork for ros2

Carla: 0.9.13 [built from source](https://carla.readthedocs.io/en/latest/build_linux/)
Carla ros bridge (0.9.13) and Ros2 [Foxy Fitzroy from source](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
-->


Follow [this guide](https://www.dhanoopbhaskar.com/blog/2020-05-07-working-with-python-3-in-ros-kinetic-or-melodic/)
to enable python3 in ros melodic

You also will need simple-pid for manual control.

`pip install --user simple-pid`

Initially, the installed directory is not in the path, so I needed to add it to the path.

`export PATH=$PATH:/home/mike/.local/lib/python3.7/site-packages`

I have an anaconda enviornment called waymo with all of the libraries needed for the sensor fusion part of the udacity course. 


# Installation

Source the carla ros bridge enviornment

`source /opt/carla-ros-bridge/melodic/setup.bash`

Then make the catkin workspace

`catkin_make`

Then create the package

`cd ~/<this project directory>/src`
`catkin_create_pkg mike_av_stack std_msgs rospy`

# How to Run

Source the current workspace:

`source devel/setup.bash`

Then the current workspace will be a child of the parent carla_ros_bridge catkin workspace.


By default, running the project mike_av_stack 

`roslaunch mike_av_stack mike_av_stack.launch`

will do the following:

- Starts basic carla_ros_bridge with an ego vehicle. You can find more info about this in the [docs.](https://carla.readthedocs.io/en/0.9.9/ros_launchs/#carla_ego_vehiclelaunch)
- Opens rviz
- Runs the sensor_fusion node

The sensor_fusion node subscribes to the topics:
- Lidar: /carla/ego_vehicle/lidar/lidar1/point_cloud

And publishes topics:
- Detections: /sensor_fusion/detection


<!-- I used [this ros question](https://answers.ros.org/question/373094/understanding-pointcloud2-data/) to understand what kind of data is in the PointCloud2 ros topic. -->


# Todo

Next steps, I want to:
- Edit/understand the sensor config files and make my own
- Make sure the ego vehicle does not change every time I run the code
- Move the rviz config file locally and clean it up a bit
- Implement detection tracking
- Implement image/camera detection
- Auto start carla and traffic gen
- Fix my traffic gen


## Installation Issues

carla-ros-bridge colcon build failed. Turns out it would somehow find my other ros2 humble installation even though there was no reference to it in PATH or PYTHONPATH. Deleting it worked

The carla-simulator installed in apt only had a python3.7.egg and building carla-ros-bridge required python3.6.
So I started from scratch, apt remove carla-sim, deleted ~/carla
Re install following very closely https://carla.readthedocs.io/en/latest/build_linux/
On make PythonAPI i get this gem: clang 8 is required, but it's not installed.

Retrying with clang set to 10 but that didn't work.

Turns out I need to edit a few files such as Setup.sh and manually set clang to 10. That works, back to carla-ros-bridge

Okay I just restarted from scratch and got another error while building carla from source. I can either use master branch and change the Setup.sh script and others from clang8 to clang10 or use dev env
Got [this error](https://github.com/carla-simulator/carla/issues/5886)
and fixed it [this way](https://stackoverflow.com/questions/40790943/usr-bin-ld-cannot-find-lstdc-for-ubuntu-while-trying-to-swift-build-perfe)

`sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/libstdc++.so`

`sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/libstdc++.so.6`

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

<!--
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
-->
