# depth_to_costmap

This repository provides a system that uses a depth image, pose input and global plan to produce steering and velocity commands to drive a car. The pose input must come at a rate higher or equal to that of the depth image.

## Brief overview:
The depth image is converted to an elevation map and projected on to the ground plane in the body reference frame. The keeps the elevation map in the body reference frame by transforming it using the pose. This way the system can maintain some history of the environment around it. The elevation map has the car's camera at the bottom-center. We can refer to this layer as the elevation layer/occupancy layer. The cost of elevation is equal to the height of a point above the minimum height set in the launch file.

The system can also take other obstacles/ boundaries and add them to the costmap. These boundary points may be obtained from a lane detection system.

An Predictive controller uses the costmap to generate the velocity and steering commands.

## Dependencies/Requirements:
We use a zed2 camera to obtain the image and an ardupilot board equipped with a ublox f9p GPS for the pose input. We use the [lane_detect](https://github.com/naughtyStark/lane_detect) repository to obtain the global plan and the lane boundaries.
1) ROS
2) ackermann-msgs
3) mavros
4) lane_detect
5) zed ros wrapper

## Usage:
launch mavros (make sure ardupilot board is connected):
```
roslaunch mavros apm.launch
```
launch zed node (make sure zed camera is connected):
```
roslaunch zed_wrapper zed2.launch
```
The above steps are required when running on the real car. If you simply want to evaluate the system with a rosbag that contains data from the ardupilot board and the zed camera, then simply playing the rosbag will also do the job.

launch global planner:
```
rosrun lane_detect lane_node_gps.py
```
wait for it to find the route (it will print that a route has been found from the car's location to the end point)
```
roslaunch depth_to_costmap depth_to_costmap.launch
```

