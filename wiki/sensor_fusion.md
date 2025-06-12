# Sensor Fusion

## Background

Sensor fusion gives the car the ability to use different sensors to detect and track objects around it such as cars, pedestrians and cyclists. A [Feature Pyramid Network or FPN](https://openaccess.thecvf.com/content_cvpr_2017/papers/Lin_Feature_Pyramid_Networks_CVPR_2017_paper.pdf) is used to detect objects from the lidar scan. An Extended Kalman Filter or EKF is used to predict, compare and track the objects from the lidar and camera results. Each new lidar scan or camera detection may introduce new objects or incorrect objects and has no context to past objects. This is why tracking is important; if an object appears in multiple consectuive lidar scans or images, then it can be concluded that the object is a real valid object.

## To Run

To disable sensor fusion, enter the following:
```bash
roslaunch mike_av_stack sensor_fusion.launch sensor_fusion:=false
```

## My Implementation

Detection is implemented using an FPN ResNet model. For lidar, detection nodes subscribe to a point cloud topic. The raw lidar point cloud topic from the carla ros bridge is only a portion of the full 360% scan, so the sensor fusion node actually subscribes to the topic output by the Mike AV stack point cloud stacker tool mentioned in the [main docs](../README.md). This full point cloud is converted to a 3 channel Birds Eye View image containing intensity, density of points, and max height of points. The BEV is then fed into the FPN resnet and detections are sent to another topic. The Sensor class has two child classes, Lidar and Camera which have callback functions to handle the sensor data then the detections. The tracking process is inside the detection callback function, which filters and associates FPN ResNet output detections with known tracked objects, and updates the known tracked object list in the Trackmanagement class. The following crudely drawn diagram shows this at a high level.
![](SensorFusionDiagram.png)

The data flow from the udacity self driving car course is all executed in one synchronous loop shown here.
![](mtt-data-flow.png)

My implementation is a massive improvement upon the udacity code as this update process is now asynchronous so camera and lidar updates can be processed in any order. I also made sure to make the track list thread safe as many callbacks can call to change this or read from it at the same time.

In addition, I have implemented and added to the sensors.json that specifies sensors for carla ros bridge. My sensor fusion code is generic and will create a unique Lidar or Camera object for every camera, and each Sensor will subscribe to its base topic. Each sensor is also given the same trackmanagement object which also has a filter and association object for the other steps.


[Back to Stack](../README.md)

# Demos

Below is a demo of camera detection. The left most display in rviz is a topic that contains images from the front camera with boxes overlayed in the image. This is purly for visualization and is only updated when a detection is in frame. The actual detection data is a Detection2DArray topic. The right screen is the manual ego vehicle control screen. The center shows just the raw point cloud from carla and the carla UI screen can be seen under it all. 
![](/wiki/Camera-detection-demo.gif)

## TODO
- Add camera network
- Fix base topic subscription
- Change lidar and camera configs for better results
- Change to YOLO v3 or v4 instead of fpn resenet
- Add visualization for tracking

