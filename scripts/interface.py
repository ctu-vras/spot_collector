#!/usr/bin/env python3
import rospy

import matplotlib.pyplot as plt
import matplotlib.lines as lines
from matplotlib.widgets import Slider, Button
import tf2_ros
import tf2_py as tf2
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
import open3d as o3d
import itertools
import genpy
from spot_collector.optimize_route import *

from geometry_msgs.msg import Point, TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from spot_msgs.srv import ArmGaze, ArmGazeRequest
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from spot_collector.srv import MultiGrasp, MultiGraspRequest


class Interface:
    def __init__(self) -> None:
        rospy.init_node("interface", anonymous=True)

        # tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # service proxys
        rospy.loginfo("Waiting for spot services")
        rospy.wait_for_service("/spot/arm_gaze")
        self.arm_gaze = rospy.ServiceProxy("/spot/arm_gaze", ArmGaze)
        rospy.wait_for_service("/spot/arm_stow")
        self.arm_stow = rospy.ServiceProxy("/spot/arm_stow", Trigger)
        rospy.wait_for_service("/spot/gripper_close")
        self.gripper_close = rospy.ServiceProxy("/spot/gripper_close", Trigger)
        rospy.wait_for_service("/multi_pick_and_place")
        self.grasp = rospy.ServiceProxy("/multi_pick_and_place", MultiGrasp)

        # publishers
        self.pcd_pub = rospy.Publisher("/detector/points", PointCloud2, queue_size=1)

        # subscribers
        self.points_subs = rospy.Subscriber(
            "/points/filtered2", PointCloud2, self.pcd_callback
        )

        # data
        self.centroids = []
        self.pcd_raw = None
        self.pcd_proc = None
        self.num_objects = 0
        self.inspect_counter = 0

        rospy.loginfo("Creating the interface")
        self.fig = plt.figure(1)
        self.fig.suptitle("spot_collector controls")
        # create matplotlib axes
        # sliders
        self.ax_thres = plt.axes([0.175, 0.15, 0.65, 0.02])
        self.ax_radius = plt.axes([0.175, 0.25, 0.65, 0.02])
        self.ax_nbout = plt.axes([0.175, 0.2, 0.65, 0.02])
        self.ax_zmax = plt.axes([0.175, 0.35, 0.65, 0.02])
        self.ax_zmin = plt.axes([0.175, 0.30, 0.65, 0.02])
        self.ax_ymax = plt.axes([0.175, 0.45, 0.65, 0.02])
        self.ax_ymin = plt.axes([0.175, 0.40, 0.65, 0.02])
        self.ax_xmax = plt.axes([0.175, 0.55, 0.65, 0.02])
        self.ax_xmin = plt.axes([0.175, 0.50, 0.65, 0.02])
        # buttons
        self.ax_crop = plt.axes([0.35, 0.05, 0.3, 0.075])
        self.ax_clust = plt.axes([0.15, 0.64, 0.33, 0.075])
        self.ax_save = plt.axes([0.52, 0.64, 0.33, 0.075])
        self.ax_ground = plt.axes([0.15, 0.74, 0.33, 0.075])
        self.ax_grasp = plt.axes([0.52, 0.74, 0.33, 0.075])
        self.ax_insp = plt.axes([0.52, 0.84, 0.33, 0.075])
        self.ax_order = plt.axes([0.15, 0.84, 0.33, 0.075])

        # create the sliders
        self.threshold_slider = Slider(
            self.ax_thres, "threshold vol", 0.0, 500.0, valinit=80.0, valstep=1.0
        )
        self.radius = Slider(
            self.ax_radius, "radius outliers", 0, 5.0, valinit=1.0, valfmt="%5.3f"
        )
        self.neib = Slider(
            self.ax_nbout, "neib outliers", 0.0, 100.0, valinit=20.0, valstep=1.0
        )
        self.zmax = Slider(
            self.ax_zmax, "Z Max", 0.0, 1.0, valinit=0.5, valfmt="%+6.3f"
        )
        self.zmin = Slider(
            self.ax_zmin, "Z Min", -1.0, 1.0, valinit=-0.2, valfmt="%+6.3f"
        )
        self.ymax = Slider(
            self.ax_ymax, "Y Max", -3.0, 3.0, valinit=1.17, valfmt="%+6.3f"
        )
        self.ymin = Slider(
            self.ax_ymin, "Y Min", -3.0, 3.0, valinit=-0.11, valfmt="%+6.3f"
        )
        self.xmax = Slider(
            self.ax_xmax, "X Max", -3.0, 3.0, valinit=1.625, valfmt="%+6.3f"
        )
        self.xmin = Slider(
            self.ax_xmin, "X Min", -3.0, 3.0, valinit=0.192, valfmt="%+6.3f"
        )

        # add functions to the sliders
        self.radius.on_changed(self._bbox)
        self.neib.on_changed(self._bbox)
        self.zmax.on_changed(self._bbox)
        self.zmin.on_changed(self._bbox)
        self.ymax.on_changed(self._bbox)
        self.ymin.on_changed(self._bbox)
        self.xmax.on_changed(self._bbox)
        self.xmin.on_changed(self._bbox)
        self.threshold_slider.on_changed(self._bbox)

        # create the buttons
        self.crop_but = Button(
            self.ax_crop, "Use Current Settings", color="white", hovercolor="green"
        )
        self.cluster_but = Button(
            self.ax_clust, "Cluster Object Points", color="white", hovercolor="green"
        )
        self.save_but = Button(
            self.ax_save, "Save Point Cloud", color="white", hovercolor="green"
        )
        self.ground_but = Button(
            self.ax_ground, "Remove ground", color="white", hovercolor="green"
        )
        self.grasp_but = Button(
            self.ax_grasp, "Send Grasp Command", color="white", hovercolor="red"
        )
        self.inspect_but = Button(
            self.ax_insp, "Inspect object", color="white", hovercolor="red"
        )
        self.order_but = Button(
            self.ax_order, "Reorder Objects", color="white", hovercolor="yellow"
        )

        # add functions to the buttons
        self.crop_but.on_clicked(self._bbox)
        self.cluster_but.on_clicked(self._cluster)
        self.save_but.on_clicked(self._save_point_cloud)
        self.ground_but.on_clicked(self._extract_soil)
        self.grasp_but.on_clicked(self._grasp)
        self.inspect_but.on_clicked(self._inspect)
        self.order_but.on_clicked(self._get_optimal_order)

        # add horizontal line between the sliders and buttons
        self.fig.add_artist(
            lines.Line2D([0.02, 0.98], [0.61, 0.61], linewidth=1, color="black")
        )

        rospy.loginfo("Interface started")

    def get_transform(self, tf_from, tf_to, out="matrix", time=None, dur=0.1):
        """returns the latest transformation between the given frames
        the result of multiplying point in frame tf_to by the output matrix is in the frame tf_from

        :param tf_from: find transform from this frame
        :param tf_to: find transform to this frame
        :param out: the return type
                    - 'matrix' - returns numpy array with the tf matrix
                    - 'tf' - returns TransformStamped
        :param time: the desired timestamp of the transform (ROS Time)
        :param dur: the timeout of the lookup (float)
        :return: as selected by out parameter or None in case of tf2 exception
                    - only ConnectivityException is logged
        """
        if time is None:
            tf_time = rospy.Time(0)
        else:
            if not isinstance(time, rospy.Time) and not isinstance(time, genpy.Time):
                raise TypeError("parameter time has to be ROS Time")
            tf_time = time

        try:
            t = self.tfBuffer.lookup_transform(
                tf_from, tf_to, tf_time, rospy.Duration(dur)
            )
        except (tf2.LookupException, tf2.ExtrapolationException):
            return None
        except tf2.ConnectivityException as ex:
            rospy.logerr(ex)
            return None

        # return the selected type
        if out == "matrix":
            return ros_numpy.numpify(t.transform)
        elif out == "tf":
            return t
        else:
            raise ValueError("argument out should be 'matrix' or 'tf'")

    def _get_optimal_order(self, event):
        reordered = test_funct(self.centroids)
        self.centroids = reordered  # apply the ordering of the detected objects

        # plot the result
        x = []
        y = []
        for c in self.centroids:
            x += [c[0]]
            y += [c[1]]

        fig = plt.figure(5)
        ax = fig.gca()
        ax.plot(x, y, "-o")
        # Start Point
        ax.plot(
            0,
            0,
            marker="o",
            markersize=10,
            markeredgecolor="red",
            markerfacecolor="black",
        )
        # First Point
        ax.plot(
            x[0],
            y[0],
            marker="o",
            markersize=10,
            markeredgecolor="red",
            markerfacecolor="green",
        )
        # End Point
        ax.plot(
            x[len(x) - 1],
            y[len(x) - 1],
            marker="o",
            markersize=10,
            markeredgecolor="red",
            markerfacecolor="green",
        )
        # Arrows to define start and direction
        ax.arrow(0, 0, x[0], y[0], color="r", length_includes_head=True, head_width=0.1)
        ax.grid(color="gray", linestyle="--", linewidth=0.5)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.show()

    def _inspect(self, event):
        # Function to pub each one of the  goal points of clusters to inspect with the arm camera
        coord_x = self.centroids[self.inspect_counter][0]
        coord_y = self.centroids[self.inspect_counter][1]
        coord_z = self.centroids[self.inspect_counter][2]

        # send gaze command
        gaze_cmd = ArmGazeRequest()
        gaze_cmd.frame_name = "vision"
        gaze_cmd.point = Point(coord_x, coord_y, coord_z)
        res = self.arm_gaze(gaze_cmd)  # returns once the arm has reached the position

        if not res.success:
            rospy.logerr("Gaze command was not successful")
            return
        else:
            rospy.loginfo("Gaze completed")

        # the object should be now approximately in the center of the image of gripper camera

        # TODO: read gripper camera image and verify if the detected object is graspable
        #       wait is probably necessary, since the camera image can be a little bit delayed
        #       multiple objects can be detected in single image, use the P matrix from sensor_msgs/CameraInfo to project the 3d point into the image

        # gaze command leaves the arm at the selected pose, stow it before returning
        # self.arm_stow()
        # the gripper has to be closed
        # self.gripper_close()
        self.inspect_counter += 1
        self.inspect_counter %= self.num_objects

    def _grasp(self, event):
        rospy.loginfo("Sending grasp request to Spot")

        points = []
        for i in range(len(self.centroids)):
            p1 = Point()
            p1.x = self.centroids[i][0]
            p1.y = self.centroids[i][1]
            p1.z = self.centroids[i][2]
            points.append(p1)

        rospy.loginfo("Detected objects: %d" % len(points))

        req = MultiGraspRequest("vision", points)
        resp = self.grasp(req)

        if resp.fail_id != -1:
            plt.text(
                2.3,
                2,
                "Grasp Fails",
                bbox={"facecolor": "r", "pad": 5},
                ha="right",
                va="top",
                transform=plt.gca().transAxes,
            )
            rospy.logerr("Failure at object with id %d" % resp.fail_id)
        else:
            rospy.loginfo("All grasps succesful")
            plt.text(
                2.3,
                2,
                "Grasp Fails",
                bbox={"facecolor": "w", "pad": 5},
                ha="right",
                va="top",
                transform=plt.gca().transAxes,
            )
        self.centroids = []

    def _extract_soil(self, event):
        if self.pcd_proc is None:
            return

        xyz = self.pcd_proc.T
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.02, ransac_n=4, num_iterations=1000
        )

        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        self.pcd_proc = np.asarray(outlier_cloud.points).T

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "vision"
        object_cloud = pc2.create_cloud_xyz32(header, outlier_cloud.points)

        self.pcd_pub.publish(object_cloud)

    def _cluster(self, event):
        self.centroids = []
        if self.pcd_proc is None:
            return

        dbscan = DBSCAN(eps=0.1, min_samples=10)
        labels = dbscan.fit_predict(self.pcd_proc.T)

        # Process the clusters
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        rospy.loginfo("Number of clusters: %d", num_clusters)

        points = []
        tfs = []
        for i in range(num_clusters):
            # -1 (noise) is ignored
            cluster_points = self.pcd_proc.T[labels == i]
            rospy.loginfo("Cluster %d: %d points", i, len(cluster_points))
            c = list(np.mean(cluster_points, axis=0))
            if len(cluster_points) <= self.threshold_slider.val:
                self.centroids.append(c)
                points += [Point(c[0], c[1], c[2])]
                tf = TransformStamped()
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = "vision"
                tf.child_frame_id = "cluster_" + str(i + 1)
                tf.transform.translation = Vector3(c[0], c[1], c[2])
                tf.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
                tfs += [tf]
        self.tfBroadcaster.sendTransform(tfs)

        self.num_objects = len(self.centroids)

    def _save_point_cloud(self, event):
        points = self.pcd_proc.T.tolist()  # list, [[x1,y1,z1],...]

        o3d_pointcloud = o3d.geometry.PointCloud()
        o3d_pointcloud.points = o3d.utility.Vector3dVector(points)

        o3d.io.write_point_cloud(
            "spot_360_filtered.pcd", o3d_pointcloud, write_ascii=True
        )

    def _bbox(self, event):
        val_rad_out = self.radius.val
        val_nb_out = self.neib.val
        zmax = self.zmax.val
        zmin = self.zmin.val
        ymax = self.ymax.val
        ymin = self.ymin.val
        xmax = self.xmax.val
        xmin = self.xmin.val

        # get position of robot in vision frame
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            tf = self.get_transform("vision", "body")
            if tf is None:
                rate.sleep()
                continue
            else:
                break
        tf[2, 3] = 0.0  # do not apply offset in z

        bounds = [[xmin, xmax], [ymin, ymax], [zmin, zmax]]  # set the bounds
        bounding_box_points = list(itertools.product(*bounds))  # create limit points

        # apply the offset
        box_arr = np.vstack((np.array(bounding_box_points).T, np.ones((1, 8))))
        box_tf = (tf @ box_arr)[:3, :]
        bounding_box_points_tf = box_tf.T.tolist()

        bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(bounding_box_points_tf)
        )

        if self.pcd_raw is None:
            return

        points = self.pcd_raw.T.tolist()  # list, [[x1,y1,z1],...]

        o3d_pointcloud = o3d.geometry.PointCloud()
        o3d_pointcloud.points = o3d.utility.Vector3dVector(points)

        o3d_pointcloud = o3d_pointcloud.crop(bounding_box)
        cloud, ind = o3d_pointcloud.remove_statistical_outlier(
            nb_neighbors=int(val_nb_out), std_ratio=val_rad_out
        )  # TODO: is the cloud same as o3d_pointcloud?
        o3d_pointcloud = o3d_pointcloud.select_by_index(ind)
        self.pcd_proc = np.asarray(o3d_pointcloud.points).T

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "vision"
        scaled_polygon_pcl = pc2.create_cloud_xyz32(header, o3d_pointcloud.points)

        self.pcd_pub.publish(scaled_polygon_pcl)

    def pcd_callback(self, msg: PointCloud2):
        self.pcd_raw = np.transpose(
            ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        )

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            plt.show()

            rate.sleep()


if __name__ == "__main__":
    i = Interface()
    i.run()
