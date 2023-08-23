# Autonomous car simulator

This project is a big one. I have already adapted the sensor fusion and localization modules into the project and I plan on adding path planning, controls and behavior in. 

I am using the Carla simulator bult on Unreal Engine to run simulations, and using ROS to send data across modules. The aim to make the project portible and to have each module abstracted such that I can run each module independent of the others and independent of Carla. My plans for the future are two fold. First, I want to run a Carla simulation on computer A, and send ros topics over a local network to computer B which then runs the modules. With this framework in place, it will be easy to impliment in a real controlled environment. For instance, raspberry pi A collects camera and lidar data from real sensors attached to a vehcile then sends ros topics to computer B also in the vehicle running the modules.

# Requirements

OS: Ubuntu 20.04\
Unreal engine: 4.26\
[carla-simulator: 0.9.13](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz)\
Ros (apt installation): Foxy\
[carla-ros-bridge/bionic: 0.9.10-1](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/)
[PCL point cloud]()

```bash
sudo apt-get install ros-foxy-pcl-ros ros-foxy-pcl-conversions ros-foxy-pcl-msgs
```

Follow [this guide](https://www.dhanoopbhaskar.com/blog/2020-05-07-working-with-python-3-in-ros-kinetic-or-melodic/)
to enable python3 in ros melodic

And [this guide](https://cyaninfinite.com/ros-cv-bridge-with-python-3/) to compile cv_bridge for python3
but change toe catkin config line to:

```bash
catkin config \
-DPYTHON_EXECUTABLE=/usr/bin/python3 \
-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so \
-DCUDA_TOOLKIT_ROOT_DIR=/home/mike/anaconda3/envs/cuda11.4/bin \
-DSWARMIO_BUILD_MOD=INSTALL \
--extend /opt/carla-ros-bridge/melodic/setup.bash
```

!!!!!!!!!!!!!!!!!!!Try to rebuild cv_bridge in this catkin ws again instead of its own.

[This link is also very useful for melodic, cv2 and anaconda](https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3)

```bash
catkin config \
-DPYTHON_EXECUTABLE=/home/mike/anaconda3/envs/waymo/bin/python3 \
-DPYTHON_INCLUDE_DIR=/home/mike/anaconda3/envs/waymo/include/python3.7m \
-DPYTHON_LIBRARY=/home/mike/anaconda3/envs/waymo/lib/libpython3.7m.so \
-DCUDA_TOOLKIT_ROOT_DIR=/home/mike/anaconda3/envs/cuda11.4/bin \
-DSWARMIO_BUILD_MOD=INSTALL \
-DSETUPTOOLS_DEB_LAYOUT=OFF \
--extend /opt/carla-ros-bridge/melodic/setup.bash
```

You also will need simple-pid for manual control.

  ```bash
  pip install --user simple-pid
  ```

Initially, the installed directory is not in the path, so I needed to add it to the path.


  ```bash
  export PATH=$PATH:/home/mike/.local/lib/python3.7/site-packages
  ```

I have an anaconda enviornment called waymo with all of the libraries needed for the sensor fusion part of the udacity course. 

You will need to compile tf2 for python3, so follow [these steps](https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/)


# Installation

Source the carla ros bridge enviornment


  ```bash
  source /opt/carla-ros-bridge/melodic/setup.bash
  ```
  

Then make the catkin workspace

  ```bash
  catkin_make
  ```

<!-- Then create the package
    This actually might only be initialize so ill have to check
    I will refine this process

  ```bash
  cd ~/<this project directory>/src
  catkin_create_pkg mike_av_stack std_msgs rospy
  ``` -->
  

# How to Run

Source the current workspace:

  ```bash
  cd .../catkin_ws/
  source devel/setup.bash
  ```

Then the current workspace will be a child of the parent carla_ros_bridge catkin workspace.

Start carla on another terminal

  ```bash
  cd /opt/carla-simulator
  ./CarlaUE4.sh
  ```
  

By default, running the project mike_av_stack 

  ```bash
  roslaunch mike_av_stack mike_av_stack.launch
  ```

will do the following:

- Starts basic carla_ros_bridge with an ego vehicle. You can find more info about this in the [docs.](https://carla.readthedocs.io/en/0.9.9/ros_launchs/#carla_ego_vehiclelaunch)
- Opens rviz
- Runs helpful tools
  - point cloud stacker
- Runs the sensor_fusion node
- Runs the localization node

# Docs

[Sensor Fusion](wiki/sensor_fusion.md)

[Localization](wiki/localization.md)


<!-- I used [this ros question](https://answers.ros.org/question/373094/understanding-pointcloud2-data/) to understand what kind of data is in the PointCloud2 ros topic. -->


# Todo

Next steps, I want to:
- Clean up config files
- Move the rviz config file locally and clean it up a bit

- Sensor Fusion
  1. Seperate topic and topic_combined into configs and put that back in
  1. Fix tracking
  1. Train a cnn on my parameters 
  1. Fix intensity scaling
  1. Add image detection
  1. Make filter generic and not dependant on trackmanagement. This way I can use for Localization

- Localization
  1. Move subscriber initialization into the scan matching constructor
  1. Implement configs
  1. Finish scan matching, and add thread safe transformation matrix
  1. Add gnss
  1. Add covariances
  1. Maybe use EKF to compare gnss and scan matching?
  1. Far off: Train a CNN to detect features like trees, buildings signs and such and create a feature map from that

- Tools
  1. Fix traffic gen


## Problems to fix

For localization, seperating the point clouds by each sensor makes no sense. So I would like to combine all sensor's point clouds to produce one global pointcloud. To do this, I would need to transform each sensor's point cloud by its extrinsic transformation matrix. 

For sensor fusion, each measurement from the detection algorithm/model is in sensor space and needs to be transformed to vehicle space.


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

  ```bash
  sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/libstdc++.so

  sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/libstdc++.so.6
  ```

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
