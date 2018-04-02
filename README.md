# hdl_localization
***hdl_localization*** is a ROS package for real-time 3D localization using a 3D LIDAR, such as velodyne HDL32e and VLP16. This package performs Unscented Kalman Filter-based pose estimation. It first estimates the sensor pose from IMU data implemented on the LIDAR, and then performs multi-threaded NDT scan matching between a globalmap point cloud and input point clouds to correct the estimated pose. IMU-based pose prediction is optional. If you disable it, the system predicts the sensor pose with the constant velocity model without IMU information.

Video:<br>
[![hdl_localization](http://img.youtube.com/vi/1EyF9kxJOqA/0.jpg)](https://youtu.be/1EyF9kxJOqA)


## Requirements
***hdl_localization*** requires the following libraries:
- OpenMP
- PCL 1.7

The following ros packages are required:
- pcl_ros
- <a href="https://github.com/koide3/ndt_omp">ndt_omp</a>

## Parameters
All parameters are listed in *launch/hdl_localization.launch* as ros params.<br>
You can specify the initial sensor pose using "2D Pose Estimate" on rviz, or using ros params (see example launch file).

## Example

Example bag files (recorded in an outdoor environment): RE
- [hdl_400.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz) (933MB)

```bash
rosparam set use_sim_time true
roslaunch hdl_localization hdl_localization.launch
```

```bash
roscd hdl_localizatrion/rviz
rviz -d hdl_localizatrion.rviz
```

```bash
rosbag play --clock hdl_400.bag
```

<img src="data/figs/localization1.png" height="256pix" /> <img src="data/figs/localization2.png" height="256pix" /> 

If it doesn't work well, change *ndt_neighbor_search_method* in *hdl_localization.launch* to "DIRECT1". It makes the scan matching significantly fast, but a little bit unstable.

## Related packages

- <a href="https://github.com/koide3/hdl_graph_slam">hdl_graph_slam</a>
- <a href="https://github.com/koide3/hdl_localization">hdl_localization</a>
- <a href="https://github.com/koide3/hdl_people_tracking">hdl_people_tracking</a>

<img src="data/figs/packages.png"/>

## Papers
Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, IEEE Transactions on Human-Machine Systems (under review) [PDF].

## Contact
Kenji Koide, Active Intelligent Systems Laboratory, Toyohashi University of Technology <a href="http://www.aisl.cs.tut.ac.jp">[URL]</a> <br>
koide@aisl.cs.tut.ac.jp

