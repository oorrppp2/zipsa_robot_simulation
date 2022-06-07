#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import actionlib
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
import tf
from tf.transformations import *
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
import geometry_msgs.msg

# from geometry_msgs.msg import TransformStamped
import tf2_ros

def main(argv):
    client = actionlib.SimpleActionClient('/plan_and_execute_pose', PlanExecutePoseAction)
    client.wait_for_server()

    goal = PlanExecutePoseGoal()
    tf_buffer = tf2_ros.Buffer()
    br = tf2_ros.TransformBroadcaster()

    listener = tf.TransformListener()

    r = rospy.Rate(10)
    # while 1:
    #     detect_pose = geometry_msgs.msg.TransformStamped()
    #     detect_pose.header.frame_id = "camera_rgb_optical_frame"
    #     detect_pose.header.stamp = rospy.Time.now()
    #     detect_pose.child_frame_id = "object_coordinate"
    #     detect_pose.transform.translation.x = -0.00209355202759769
    #     detect_pose.transform.translation.y = 0.17811315021118168
    #     detect_pose.transform.translation.z = 0.9182939423939185

    #     detect_pose.transform.rotation.x = 0.4260005463764649
    #     detect_pose.transform.rotation.y = 0.8828
    #     detect_pose.transform.rotation.z = -0.19026
    #     detect_pose.transform.rotation.w = 0.053722

    #     br.sendTransform(detect_pose)
    try:
        detect_pose = geometry_msgs.msg.TransformStamped()
        detect_pose.header.frame_id = "camera_rgb_optical_frame"
        detect_pose.header.stamp = rospy.Time.now()
        detect_pose.child_frame_id = "object_coordinate"
        detect_pose.transform.translation.x = -0.00209355202759769
        detect_pose.transform.translation.y = 0.17811315021118168
        detect_pose.transform.translation.z = 0.9182939423939185

        detect_pose.transform.rotation.x = 0.4260005463764649
        detect_pose.transform.rotation.y = 0.8828
        detect_pose.transform.rotation.z = -0.19026
        detect_pose.transform.rotation.w = 0.053722
        
        count = 0
        while 1:
            count += 1
            detect_pose.header.stamp = rospy.Time.now()
            br.sendTransform(detect_pose)
            r.sleep()
            try:
                (trans,rot) = listener.lookupTransform('/base_footprint', '/object_coordinate', rospy.Time(0))
                break
            except:
                continue
        
        print("count : ", count)
        # (trans,rot) = listener.lookupTransform('/camera_rgb_optical_frame', '/object_coordinate', rospy.Time(0))
        print("trans: ", trans)
        print("rot: ", rot)
        # convert detect_pose with reference base_footprint

        goal.target_pose.header.frame_id = "base_footprint"
        goal.target_pose.pose.position.x = trans[0]
        goal.target_pose.pose.position.y = trans[1]
        goal.target_pose.pose.position.z = trans[2]

        goal.target_pose.pose.orientation.x = rot[0]
        goal.target_pose.pose.orientation.y = rot[1]
        goal.target_pose.pose.orientation.z = rot[2]
        goal.target_pose.pose.orientation.w = rot[3]

        roll, pitch, yaw = euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
        print(roll, pitch, yaw)

        if yaw < -math.pi / 2.0:
            yaw += math.pi
        elif yaw > math.pi / 2.0:
            yaw -= math.pi
        quat = quaternion_from_euler(roll, pitch, yaw)

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        roll, pitch, yaw = euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
        print(roll, pitch, yaw)

        # goal.target_pose.pose.orientation.x = -0.449
        # goal.target_pose.pose.orientation.y = -0.005
        # goal.target_pose.pose.orientation.z = 0.890
        # goal.target_pose.pose.orientation.w = 0.074
        # goal.target_pose.pose.orientation.x = 0
        # goal.target_pose.pose.orientation.y = 0
        # goal.target_pose.pose.orientation.z = 0.0
        # goal.target_pose.pose.orientation.w = 1.0

    except ValueError:
        quit()

    client.send_goal(goal)
    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_pose', anonymous=False)

    # if len(sys.argv) != 8:
    #     print "Usage: rosrun living_lab_robot_moveit_client test_pose <reference_link> <x> <y> <z> <r (deg)> <p (deg)> <y (deg)>"
    #     exit(-1)

    m = main(sys.argv[1:])

    # rospy.spin()