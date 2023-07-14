# Robot Spot as a tool for collecting items
ROS package for using the Boston Dynamics Spot equiped with the Spot arm for collecting objects placed on the ground around it.

This package requires a [custom version](https://github.com/ctu-vras/spot_ros) of the ROS driver for Spot

## Usage
On the computer connected to the robot, use

```console
$ roslaunch spot_collector collector.launch
```

The node for detecting graspable objects requires user interface, on another computer equiped with display run

```console
$ rosrun spot_collector Interf_real_time.py
```

The following interface will be opened:

![User interface](https://github.com/ctu-vras/spot_collector/assets/127795959/2b07b3fc-2551-45b5-8f9a-6a2d79128775)

In the user interface, set the detection area with the sliders. Afterwards, press button **Extract Soil** to remove points belonging to the ground from the point cloud. Button **Get Clusters** runs the detection, new frames will be added to the transformation tree. Visual verification of the objects can be performed with the **Pub Pose Arm Inspect** button. Before grasping, the detected objects can be sorted (button **Get Optimal Sequence**) to make the grasping procedure faster. When the objects are correctly detected, grasp command can be issued with the **Send Grasp Object** button.
