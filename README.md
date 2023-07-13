# spot_collector
ROS package for using the Boston Dynamics Spot equiped with the Spot arm for collecting objects placed on the ground around it.

This package requires a [custom version](https://github.com/ctu-vras/spot_ros/tree/dev) of the ROS driver for Spot

## Usage
On the computer connected to the robot, use

```console
$ roslaunch spot_collector collector.launch
```

The node for detecting graspable objects requires user interface, on another computer equiped with display run

```console
$ rosrun spot_collector Interf_real_time.py
```
