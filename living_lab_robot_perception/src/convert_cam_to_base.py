#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
# from cafe_robot_moveit.msg import PlanExecutePoseGoal
# from cafe_robot_moveit.msg import PlanExecutePoseAction, PlanExecutePoseFeedback, PlanExecutePoseResult
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
import tf
from tf.transformations import *
from std_srvs.srv import Empty
from control_msgs.srv import QueryTrajectoryState

# from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
import geometry_msgs.msg
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from pose_msg.msg import Result
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import String
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2

# import pcl
# import pcl_helper


class Cam_to_Base():
    def __init__(self):
        rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        self.clear_octomap()
        # exit(0)
        self.plan_client = actionlib.SimpleActionClient('/plan_and_execute_pose', PlanExecutePoseAction)
        self.plan_client.wait_for_server()
        self.constrained_plan_client = actionlib.SimpleActionClient('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction)
        self.constrained_plan_client.wait_for_server()
        self.remove_switching_pub = rospy.Publisher('/remove_points_switch', String, queue_size=1)

        self.goal = PlanExecutePoseGoal()
        self.sub_goal = PlanExecutePoseGoal()
        self.tf_buffer = tf2_ros.Buffer()
        self.br = tf2_ros.TransformBroadcaster()
        self.r = rospy.Rate(10)

        self.listener = tf.TransformListener()

        self.estimated_pose_sub = rospy.Subscriber('/contact_graspnet/results', Result, self.Pose_callback)

        rospy.loginfo("Initialized...")

    def Pose_callback(self, msg):
        estimated_pose = msg.pose
        estimated_pose = np.asarray(estimated_pose).reshape(4,4)
        print(estimated_pose)
        
        rotation_mat = np.zeros((4,4))
        rotation_mat[:3,:3] = estimated_pose[:3,:3]
        rotation_mat[3,3] = 1

        # Rotate R x: -90, z: -90
        z_rotation_mat = np.asarray([[math.cos(-math.pi/2.0), -math.sin(-math.pi/2.0), 0, 0],
                                    [math.sin(-math.pi/2.0), math.cos(-math.pi/2.0), 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        
        x_rotation_mat = np.asarray([[1, 0, 0, 0],
                                    [0, math.cos(-math.pi/2.0), -math.sin(-math.pi/2.0), 0],
                                    [0, math.sin(-math.pi/2.0), math.cos(-math.pi/2.0), 0],
                                    [0, 0, 0, 1]])
        
        y_rotation_mat = np.asarray([[math.cos(-math.pi/2.0), 0, math.sin(-math.pi/2.0), 0],
                                    [0, 1, 0, 0],
                                    [-math.sin(-math.pi/2.0), 0, math.cos(-math.pi/2.0), 0],
                                    [0, 0, 0, 1]])
        
        rotation_mat = np.dot(rotation_mat, x_rotation_mat)
        rotation_mat = np.dot(rotation_mat, z_rotation_mat)
        # rotation_mat = np.dot(rotation_mat, z_rotation_mat)
        # rotation_mat = np.dot(rotation_mat, y_rotation_mat)
        # rotation_mat = np.dot(z_rotation_mat, rotation_mat)
        # rotation_mat = np.dot(x_rotation_mat, rotation_mat)

        estimated_pose[:3,:3] = rotation_mat[:3,:3]
        print("rotated matrix")
        print(estimated_pose)

        quat = quaternion_from_matrix(estimated_pose)
        # quat = quat / np.linalg.norm(quat)
        detect_pose = geometry_msgs.msg.TransformStamped()
        detect_pose.header.frame_id = "camera_rgb_optical_frame"
        detect_pose.header.stamp = rospy.Time.now()
        detect_pose.child_frame_id = "object_coordinate"
        detect_pose.transform.translation.x = estimated_pose[0,3]
        detect_pose.transform.translation.y = estimated_pose[1,3]
        detect_pose.transform.translation.z = estimated_pose[2,3]

        detect_pose.transform.rotation.x = quat[0]
        detect_pose.transform.rotation.y = quat[1]
        detect_pose.transform.rotation.z = quat[2]
        detect_pose.transform.rotation.w = quat[3]

        while 1:
            detect_pose.header.stamp = rospy.Time.now()
            self.br.sendTransform(detect_pose)
            self.r.sleep()
            try:
                (trans,rot) = self.listener.lookupTransform('/base_footprint', '/object_coordinate', rospy.Time(0))
                print(trans, rot)
                break
            except:
                continue

        # print("trans: ", trans)
        # print("rot: ", rot)
        # convert detect_pose with reference base_footprint

        self.goal.target_pose.header.frame_id = "base_footprint"
        self.goal.target_pose.pose.position.x = trans[0]
        self.goal.target_pose.pose.position.y = trans[1]
        self.goal.target_pose.pose.position.z = trans[2]

        self.goal.target_pose.pose.orientation.x = rot[0]
        self.goal.target_pose.pose.orientation.y = rot[1]
        self.goal.target_pose.pose.orientation.z = rot[2]
        self.goal.target_pose.pose.orientation.w = rot[3]

        roll, pitch, yaw = euler_from_quaternion([self.goal.target_pose.pose.orientation.x, self.goal.target_pose.pose.orientation.y,
        self.goal.target_pose.pose.orientation.z, self.goal.target_pose.pose.orientation.w])

        if yaw < -math.pi / 2.0:
            yaw += math.pi
        elif yaw > math.pi / 2.0:
            yaw -= math.pi
        quat = quaternion_from_euler(roll, pitch, yaw)

        self.goal.target_pose.pose.orientation.x = quat[0]
        self.goal.target_pose.pose.orientation.y = quat[1]
        self.goal.target_pose.pose.orientation.z = quat[2]
        self.goal.target_pose.pose.orientation.w = quat[3]

        rospy.wait_for_service('/arm_controller/query_state')
        try:
            query_state = rospy.ServiceProxy('/arm_controller/query_state', QueryTrajectoryState)
            resp = query_state(rospy.Time.now())
        except rospy.ServiceException(e):
            print("Service call failed: %s"%e)

        constrained_plan_goal = PlanExecutePoseConstraintsGoal()
        constrained_plan_goal.target_pose = self.goal.target_pose

        joint_names = resp.name
        joint_positions = resp.position

        # joint_constraint = JointConstraint()
        # joint_constraint.joint_name = 'arm1_joint'  # joint_names[2]
        # joint_constraint.position = joint_positions[2]
        # joint_constraint.tolerance_above = (np.pi / 6.0)
        # joint_constraint.tolerance_below = (np.pi / 6.0)
        # joint_constraint.weight = 1.0
        # constrained_plan_goal.joint_constraints.append(joint_constraint)

        # joint_constraint = JointConstraint()
        # joint_constraint.joint_name = 'arm4_joint'  # joint_names[5]
        # joint_constraint.position = joint_positions[5]
        # joint_constraint.tolerance_above = (np.pi / 6.0)
        # joint_constraint.tolerance_below = (np.pi / 6.0)
        # joint_constraint.weight = 1.0
        # constrained_plan_goal.joint_constraints.append(joint_constraint)

        # joint_constraint = JointConstraint()
        # joint_constraint.joint_name = 'arm6_joint'  # joint_names[7]
        # joint_constraint.position = joint_positions[7]
        # joint_constraint.tolerance_above = (np.pi / 6.0)
        # joint_constraint.tolerance_below = (np.pi / 6.0)
        # joint_constraint.weight = 1.0
        # constrained_plan_goal.joint_constraints.append(joint_constraint)

        # self.clear_octomap()
        # self.constrained_plan_client.send_goal(constrained_plan_goal)
        # self.constrained_plan_client.wait_for_result()

        # Sub goal

        R = quaternion_matrix(quat)[:3,:3]
        XL2 = np.matmul(R.T, trans)
        XL2[0] -= 0.1

        sub_trans = np.matmul(R, XL2)

        self.sub_goal.target_pose.header.frame_id = "base_footprint"
        self.sub_goal.target_pose.pose.position.x = sub_trans[0]
        self.sub_goal.target_pose.pose.position.y = sub_trans[1]
        self.sub_goal.target_pose.pose.position.z = sub_trans[2]

        self.sub_goal.target_pose.pose.orientation.x = quat[0]
        self.sub_goal.target_pose.pose.orientation.y = quat[1]
        self.sub_goal.target_pose.pose.orientation.z = quat[2]
        self.sub_goal.target_pose.pose.orientation.w = quat[3]

        self.clear_octomap()
        self.plan_client.send_goal(self.sub_goal)
        self.plan_client.wait_for_result()

        self.clear_octomap()
        self.constrained_plan_client.send_goal(constrained_plan_goal)
        self.constrained_plan_client.wait_for_result()

    # for initial_pose in initial_poses:
    #     remove_switching_pub.publish("off")
    #     print("initial_pose")
    #     print(initial_pose)
    #     # if initial_pose[2,3] < 0.12:
    #     #     continue
    #     # if initial_pose[2,3] > 0.9:
    #     #     continue
    #     # if initial_pose[0,3] < 0:
    #     #     continue
    #     if np.sum(initial_pose) == 0:
    #         continue
    #     M = np.matmul(estimated_pose, initial_pose)
    #     quat = quaternion_from_matrix(M)

    #     try:
    #         detect_pose = geometry_msgs.msg.TransformStamped()
    #         detect_pose.header.frame_id = "camera_depth_optical_frame"
    #         detect_pose.header.stamp = rospy.Time.now()
    #         detect_pose.child_frame_id = "object_coordinate"
    #         detect_pose.transform.translation.x = M[0,3]
    #         detect_pose.transform.translation.y = M[1,3]
    #         detect_pose.transform.translation.z = M[2,3]

    #         detect_pose.transform.rotation.x = quat[0]
    #         detect_pose.transform.rotation.y = quat[1]
    #         detect_pose.transform.rotation.z = quat[2]
    #         detect_pose.transform.rotation.w = quat[3]

    #         count = 0
    #         while 1:
    #             count += 1
    #             detect_pose.header.stamp = rospy.Time.now()
    #             br.sendTransform(detect_pose)
    #             r.sleep()
    #             try:
    #                 (trans,rot) = listener.lookupTransform('/base_footprint', '/object_coordinate', rospy.Time(0))
    #                 break
    #             except:
    #                 continue

    #         # print("trans: ", trans)
    #         # print("rot: ", rot)
    #         # convert detect_pose with reference base_footprint

    #         goal.target_pose.header.frame_id = "base_footprint"
    #         goal.target_pose.pose.position.x = trans[0]
    #         goal.target_pose.pose.position.y = trans[1]
    #         goal.target_pose.pose.position.z = trans[2]

    #         goal.target_pose.pose.orientation.x = rot[0]
    #         goal.target_pose.pose.orientation.y = rot[1]
    #         goal.target_pose.pose.orientation.z = rot[2]
    #         goal.target_pose.pose.orientation.w = rot[3]

    #         roll, pitch, yaw = euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
    #         goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
    #         # print(roll, pitch, yaw)

    #         if yaw < -math.pi / 2.0:
    #             yaw += math.pi
    #         elif yaw > math.pi / 2.0:
    #             yaw -= math.pi
    #         quat = quaternion_from_euler(roll, pitch, yaw)

    #         goal.target_pose.pose.orientation.x = quat[0]
    #         goal.target_pose.pose.orientation.y = quat[1]
    #         goal.target_pose.pose.orientation.z = quat[2]
    #         goal.target_pose.pose.orientation.w = quat[3]

    #         R = quaternion_matrix(quat)[:3,:3]
    #         XL2 = np.matmul(R.T, trans)
    #         XL2[0] -= 0.1

    #         sub_trans = np.matmul(R, XL2)

    #         sub_goal.target_pose.header.frame_id = "base_footprint"
    #         sub_goal.target_pose.pose.position.x = sub_trans[0]
    #         sub_goal.target_pose.pose.position.y = sub_trans[1]
    #         sub_goal.target_pose.pose.position.z = sub_trans[2]

    #         sub_goal.target_pose.pose.orientation.x = quat[0]
    #         sub_goal.target_pose.pose.orientation.y = quat[1]
    #         sub_goal.target_pose.pose.orientation.z = quat[2]
    #         sub_goal.target_pose.pose.orientation.w = quat[3]

    #     except ValueError:
    #         quit()

    #     print(sub_goal)
    #     plan_client.send_goal(sub_goal)
    #     plan_client.wait_for_result()

    #     if plan_client.get_result().result == True:
    #         remove_switching_pub.publish("on")
    #         rospy.loginfo("Clearing Octomap")
    #         clear_octomap()
    #         rospy.wait_for_service('/arm_controller/query_state')
    #         try:
    #             query_state = rospy.ServiceProxy('/arm_controller/query_state', QueryTrajectoryState)
    #             resp = query_state(rospy.Time.now())
    #         except rospy.ServiceException, e:
    #             print "Service call failed: %s"%e

    #         constrained_plan_goal = PlanExecutePoseConstraintsGoal()
    #         constrained_plan_goal.target_pose = goal.target_pose

    #         joint_names = resp.name
    #         joint_positions = resp.position

    #         joint_constraint = JointConstraint()
    #         joint_constraint.joint_name = 'arm1_joint'  # joint_names[2]
    #         joint_constraint.position = joint_positions[2]
    #         joint_constraint.tolerance_above = (np.pi / 6.0)
    #         joint_constraint.tolerance_below = (np.pi / 6.0)
    #         joint_constraint.weight = 1.0
    #         constrained_plan_goal.joint_constraints.append(joint_constraint)

    #         joint_constraint = JointConstraint()
    #         joint_constraint.joint_name = 'arm4_joint'  # joint_names[5]
    #         joint_constraint.position = joint_positions[5]
    #         joint_constraint.tolerance_above = (np.pi / 6.0)
    #         joint_constraint.tolerance_below = (np.pi / 6.0)
    #         joint_constraint.weight = 1.0
    #         constrained_plan_goal.joint_constraints.append(joint_constraint)

    #         joint_constraint = JointConstraint()
    #         joint_constraint.joint_name = 'arm6_joint'  # joint_names[7]
    #         joint_constraint.position = joint_positions[7]
    #         joint_constraint.tolerance_above = (np.pi / 6.0)
    #         joint_constraint.tolerance_below = (np.pi / 6.0)
    #         joint_constraint.weight = 1.0
    #         constrained_plan_goal.joint_constraints.append(joint_constraint)

    #         constrained_plan_client.send_goal(constrained_plan_goal)
    #         constrained_plan_client.wait_for_result()

    #         print constrained_plan_client.get_result()
    #         if constrained_plan_client.get_result().result == True:
    #             break
    #         else:   # Control the manipulator back to the home position.
    #             client = actionlib.SimpleActionClient('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction)
    #             client.wait_for_server()

    #             named_goal = PlanExecuteNamedPoseGoal()
    #             named_goal.target_name = "home"

    #             client.send_goal(named_goal)
    #             client.wait_for_result()

    #     print("="*50)

    # remove_switching_pub.publish("off")

if __name__ == '__main__':
    rospy.init_node('convert_cam_to_base', anonymous=False)

    convert_class = Cam_to_Base()
    rospy.spin()
