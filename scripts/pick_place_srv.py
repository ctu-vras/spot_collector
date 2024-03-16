#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2
import actionlib
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from spot_msgs.msg import (
    TrajectoryAction,
    TrajectoryGoal,
)
from std_srvs.srv import Trigger, TriggerRequest
from spot_msgs.srv import Grasp3d, Grasp3dRequest
from spot_msgs.srv import ArmCartesianTrajectory, ArmCartesianTrajectoryRequest

from spot_collector.srv import MultiGrasp, MultiGraspRequest, MultiGraspResponse


N_POINTS = 10


def quat_interp(q1: Quaternion, q2: Quaternion, q3: Quaternion, N: int, N3: int):
    """Slerp quaternion interpolation, allows to set the direction of rotation by q2

    Args:
        q1: start orientation
        q2: orientation between start and end
        q3: end orientation
        N: number of the quaternions at output
        N3: position of q2 in output

    Returns:
        geometry_msgs/Quaternion[] interpolated quaternions
    """
    r1 = R.from_quat([q1.x, q1.y, q1.z, q1.w])
    r2 = R.from_quat([q2.x, q2.y, q2.z, q2.w])
    r3 = R.from_quat([q3.x, q3.y, q3.z, q3.w])
    times = np.arange(0, N)
    key_rots = R.from_quat([r1.as_quat(), r2.as_quat(), r3.as_quat()])
    slerp = Slerp([0, N3 - 1, N - 1], key_rots)
    interp_rots = slerp(times)
    out = []
    for r in interp_rots:
        q = r.as_quat()
        out += [Quaternion(q[0], q[1], q[2], q[3])]
    return out


class PickPlace:
    """Service for grasping multiple objects from the specified coordinates"""

    def __init__(self):
        rospy.init_node("multi_pick_and_place")

        # tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # generate circle arm trajectory from carry to container
        # use 3 quaternions to force the arm to rotate counterclockwise to avoid limit of the first joint
        q1 = Quaternion(
            0.7284430861473083,
            0.017499305307865143,
            -0.03009978123009205,
            0.684222936630249,
        )
        q2 = Quaternion(
            -0.25776058435440063,
            -0.6635450124740601,
            0.6546700596809387,
            0.25431299209594727,
        )
        q3 = Quaternion(
            0.15078383684158325,
            -0.6926872134208679,
            0.6872392296791077,
            -0.15859219431877136,
        )
        # start - close to the carry position
        p1 = np.array([[0.8762252330780029], [0.0], [0.6058109402656555]])
        # end - above the container
        p3 = np.array(
            [[-0.2871779799461365], [0.023444153368473053], [0.6058109402656555]]
        )
        self.traj = [Pose(Point(p1[0, 0], p1[1, 0], p1[2, 0]), q1)]
        # compute center of the circle
        center = (p1 + p3) / 2
        center[1, 0] = 0.0  # keep the center aligned with the body axis
        p1_s = p1 - center
        p3_s = p3 - center
        norm = np.array([[0], [0], [1]])
        # compute rotation between the start and end of the trajectory
        angle = float(
            np.arctan2(
                np.matmul(np.cross(p1_s, p3_s, axis=0).T, norm), np.matmul(p1_s.T, p3_s)
            )
        )
        angle_inc = angle / (N_POINTS - 1)
        s = np.sin(angle_inc)
        c = np.cos(angle_inc)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        point = p1_s
        q_int = quat_interp(q1, q2, q3, N_POINTS, 7)
        for i in range(N_POINTS - 1):
            point = np.matmul(rot, point)
            tmp = point + center
            self.traj += [Pose(Point(tmp[0, 0], tmp[1, 0], tmp[2, 0]), q_int[i + 1])]

        # the pose from which Spot started to grasp the objects
        self.init_pose = None

        rospy.loginfo("Waiting for Spot services")
        rospy.wait_for_service("/spot/grasp_3d")
        rospy.wait_for_service("/spot/arm_cartesian_trajectory")
        rospy.wait_for_service("/spot/gripper_open")
        rospy.wait_for_service("/spot/gripper_close")
        rospy.wait_for_service("/spot/arm_stow")
        self.pickup_proxy = rospy.ServiceProxy("/spot/grasp_3d", Grasp3d)
        self.trajectory_proxy = rospy.ServiceProxy(
            "/spot/arm_cartesian_trajectory", ArmCartesianTrajectory
        )
        self.open_proxy = rospy.ServiceProxy("/spot/gripper_open", Trigger)
        self.close_proxy = rospy.ServiceProxy("/spot/gripper_close", Trigger)
        self.stow_proxy = rospy.ServiceProxy("/spot/arm_stow", Trigger)
        self.trajectory_client = actionlib.SimpleActionClient(
            "/spot/trajectory", TrajectoryAction
        )
        self.trajectory_client.wait_for_server()

        # start the service
        self.service = rospy.Service("/multi_pick_and_place", MultiGrasp, self.run)

        rospy.loginfo("Ready to pick up objects")

    def get_transform(self, to_frame: str, from_frame: str):
        """Get transform from the tf buffer, the transform shows position of from_frame in to_frame

        Args:
            to_frame: transform to this frame
            from_frame: transform from this frame

        Returns:
            geometry_msgs/TransformStamped or None
        """
        tf_time = rospy.Time(0)
        try:
            t = self.tfBuffer.lookup_transform(
                to_frame, from_frame, tf_time, rospy.Duration(0.1)
            )
        except (tf2.LookupException, tf2.ExtrapolationException):
            return None
        except tf2.ConnectivityException as ex:
            rospy.logerr(ex)
            return None

        return t

    def run(self, req: MultiGraspRequest):
        """Grasping service handler. Command Spot to grasp objects specified in the request

        Args:
            req: the service request containing positions of the objects to be grasped

        Returns:
            spot_collector/MultiGraspResponse result of the grasping procedure
        """
        # get the initial pose of the robot
        rospy.loginfo("Trying to obtain the starting pose of the robot")
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            tf = self.get_transform("vision", "body")
            if tf is None:
                rate.sleep()
                continue
            else:
                break

        points = req.coords
        frame = req.fixed_frame
        index = 0
        abort = False
        rospy.loginfo("Received grasp request with %d objects" % (len(points)))
        for p in points:
            # build grasp request
            req_pick = Grasp3dRequest(frame, [p.x, p.y, p.z])
            # send pickup request
            rospy.loginfo("Sending pickup request")
            resp = self.pickup_proxy(req_pick)
            if resp.success:
                # move the hand above the container
                r = ArmCartesianTrajectoryRequest()
                r.root_frame = "body"
                r.traj_time = 2.0
                r.poses = self.traj
                rospy.loginfo("Sending trajectory request")
                resp = self.trajectory_proxy(r)
                if not resp.success:
                    self.trajectory_fallback(True)
                    abort = True
                    break

                # open the gripper
                self.open_proxy(TriggerRequest())
                time.sleep(0.5)

                # move the arm back to front to avoid singular position
                r = ArmCartesianTrajectoryRequest()
                r.root_frame = "body"
                r.traj_time = 2.0
                r.poses = self.traj[::-1]
                rospy.loginfo("Sending trajectory request")
                resp = self.trajectory_proxy(r)
                if not resp.success:
                    self.trajectory_fallback(False)
                    abort = True
                    break

                self.close_proxy()
                self.stow_proxy()
                time.sleep(0.5)
                rospy.loginfo("Grasp successful")
            else:
                rospy.logwarn("Grasp %d failed, aborting" % index)
                abort = True
                break
            index += 1
        if abort:
            msg = "Execution stopped at first failure [object with index %d]" % index
            success = False
        else:
            msg = "All grasps successful"
            success = True
            index = -1  # no grasp failed

        # return back to the starting pose
        rospy.loginfo("Returning the robot to the starting pose")
        v3 = tf.transform.translation
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "vision"
        p.pose.position = Point(v3.x, v3.y, v3.z)
        p.pose.orientation = tf.transform.rotation
        goal = TrajectoryGoal()
        goal.target_pose = p
        goal.duration.data.secs = 10
        # 10 seconds time-out should be enough for the robot to return
        goal.precise_positioning = True
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        res = self.trajectory_client.get_result()
        if res.success:
            rospy.loginfo("Robot returned to the starting pose")
        else:
            rospy.logerr("Robot could not return to the starting pose")
            rospy.logerr("Reason: %s" % res.message)

        return MultiGraspResponse(success, index, msg)

    def trajectory_fallback(self, holding: bool):
        """If the arm trajectory fails, empty the gripper and stow the arm
        :param bool holding: is the gripper holding an object?
        :return: None
        """
        rospy.logerr("Trajectory command failed, stowing arm")
        if holding:
            rospy.logerr("Emptying the gripper before stowing the arm")
            self.open_proxy()
            rospy.sleep(2)
            self.close_proxy()
        self.stow_proxy()


if __name__ == "__main__":
    pick_server = PickPlace()
    rospy.spin()
