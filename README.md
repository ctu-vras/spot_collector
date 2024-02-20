# Robot Spot as a tool for collecting items
ROS package for using the Boston Dynamics Spot equiped with the Spot arm for collecting objects placed on the ground around it.

This package requires a [custom version](https://github.com/ctu-vras/spot_ros) of the ROS driver for Spot

## User Guide

> [!NOTE]  
> Python3 version is requiered since the Open3D library uses it.

1. Place the robot in an area with items to collect
1. Launch custom ROS driver (`roslaunch spot_ros driver.launch`), power on the robot and issue stand command
1. On the NUC connected to the robot: `roslaunch spot_collector collector.launch`
1. On another computer with display, launch the user interface: `rosrun spot_collector Interf_real_time.py`
   
    The following interface will be opened:
   
    ![User interface](https://github.com/ctu-vras/spot_collector/assets/127795959/e510dd53-439e-4dd0-997c-f9f864fd94a9)

1. With the sliders, set the area in which objects should be collected - the settings can be verified by looking at PointCloud2 messages published on topic */detector/points*. The default configuration delimits an area of 1 square meter, with a maximum area of 6 by 6 meters around the robot; the default values and limits can be modified in the script ([interface.py](scripts/interface.py#L86-L112)). It is also possible to use the current values set by the sliders by pressing the **Use current settings** button.
1. Remove points belonging to ground. It can be either done by setting the limits in the z-axis (with the two sliders) or by pressing the button **Remove ground**.
1. Detect the individual objects by clustering the remaining points - press button **Cluster Object Points**. New frames will be added to the tf tree. The detected objects can be therefore seen in Rviz.
1. Before grasping, the objects can be sorted into an optimal sequence with button **Reorder Objects**. The grasp command for all detected objects is sent with the button **Send Grasp Command**. Robot then collects all the detected objects.
1. If one of the grasps fails, all following grasps are aborted, failure is indicated in the top right corner of the user interface. New detection is required to grasp the remaining objects (start again from step 5.).
