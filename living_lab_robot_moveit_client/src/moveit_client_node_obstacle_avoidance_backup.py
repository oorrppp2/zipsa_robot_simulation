#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseFeedback, PlanExecutePoseResult
from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseFeedback, PlanExecuteNamedPoseResult
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsFeedback, PlanExecutePoseConstraintsResult
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
import shape_msgs.msg
import object_recognition_msgs.srv as object_recognition_srvs
from moveit_python import PlanningSceneInterface

COLLISION_OBJECT_TOPIC = "/collision_object"
OBJECT_INFORMATION_TOPIC = "/get_object_info"

class MoveitClientNode:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

#	self.scene.removeCollisionObject('table')
#	table_size_x = 0.5
#	table_size_y = 1
#	table_size_z = 0.03
#	table_x = 0.8
#	table_y = 0
#	table_z = 0.6
#	self.scene.addBox('table', table_size_x, table_size_y, table_size_z, table_x, table_y, table_z)
#	self.scene.removeCollisionObject('table')

        self.contraints = Constraints()

        is_initialized = False
        while(not is_initialized):
            try:
                self.group = moveit_commander.MoveGroupCommander("arm")
                is_initialized = True
            except RuntimeError:
                is_initialized = False
                rospy.sleep(0.5)

	info_response = object_recognition_srvs.GetObjectInformationResponse()

	pub_collision_object = rospy.Publisher(COLLISION_OBJECT_TOPIC,
                                           moveit_msgs.msg.CollisionObject, queue_size=10,
                                           latch = True)
	collision_object = moveit_msgs.msg.CollisionObject()
	collision_object.header.frame_id = "base_footprint";
	collision_object.id = info_response.information.name

	print("collision_object.id : " + collision_object.id)
	#collision_object.id = "box1";

	object_shape = shape_msgs.msg.SolidPrimitive()
	object_shape.type = object_shape.BOX;
	object_shape.dimensions.append(0.2) # BOX x
        object_shape.dimensions.append(0.8) # BOX y
        object_shape.dimensions.append(0.45) # BOX z
        collision_object.primitives.append(object_shape)
	shape_pose = geometry_msgs.msg.Pose()
        shape_pose.position.x = 0.5
        shape_pose.position.y = 0.0
        shape_pose.position.z = 0.0 + object_shape.dimensions[0] / 2
        shape_pose.orientation.w = 1.0
        collision_object.primitive_poses.append(shape_pose)
        ''' add a mesh from the household objects database '''
	print("info_response.information.ground_truth_mesh type : " + str(type(info_response.information.ground_truth_mesh)))
        collision_object.meshes.append(info_response.information.ground_truth_mesh)
        mesh_pose = geometry_msgs.msg.Pose()
        mesh_pose.position.x = 0.5
        mesh_pose.position.y = 0.0
        mesh_pose.position.z = 0.0
        mesh_pose.orientation.w = 1.0
        collision_object.mesh_poses.append(mesh_pose)
        
        collision_object.operation = moveit_msgs.msg.CollisionObject.ADD
#        rospy.loginfo('Adding object ...')
	pub_collision_object.publish(collision_object)


        rospy.loginfo("Initialized...")
        self.traj_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        rospy.loginfo("============ Reference planning frame: %s" % self.group.get_planning_frame())
        rospy.loginfo("============ Reference end_effector link: %s" % self.group.get_end_effector_link())

        self.action_plan_execute_pose = actionlib.SimpleActionServer('/plan_and_execute_pose', PlanExecutePoseAction, execute_cb=self.plan_execute_pose_cb, auto_start = False)
        self.action_plan_execute_pose.start()

        self.action_plan_execute_named_pose = actionlib.SimpleActionServer('/plan_and_execute_named_pose', PlanExecuteNamedPoseAction, execute_cb=self.plan_execute_named_pose_cb, auto_start = False)
        self.action_plan_execute_named_pose.start()

        self.action_plan_execute_pose_w_constraints = actionlib.SimpleActionServer('/plan_and_execute_pose_w_joint_constraints', PlanExecutePoseConstraintsAction, execute_cb=self.plan_execute_pose_constraints_cb, auto_start = False)
        self.action_plan_execute_pose_w_constraints.start()
        rospy.loginfo('%s ready...'%rospy.get_name())

    def plan_execute_pose_cb(self, goal):
        feedback = PlanExecutePoseFeedback()
        result = PlanExecutePoseResult()
        result.result = True

        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()

        try:
            self.group.set_pose_target(goal.target_pose)
        except MoveItCommanderException:
            result.result = False
            return

        rospy.loginfo('Planning goal pose...')
        plan1 = self.group.plan()

        if len(plan1.joint_trajectory.points) == 0:
            result.result = False
            return

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.traj_publisher.publish(display_trajectory)

        rospy.sleep(0.5)

        rospy.loginfo('Start moving...')
        self.group.go(wait=True)
        rospy.sleep(2.0)

        rospy.loginfo('Planning goal pose succeeded.')
        self.action_plan_execute_pose.set_succeeded(result)

    def plan_execute_named_pose_cb(self, goal):
        feedback = PlanExecuteNamedPoseFeedback()
        result = PlanExecuteNamedPoseResult()
        result.result = True

        self.group.clear_pose_targets()
        #self.group.set_start_state_to_current_state()

        try:
            self.group.set_named_target(goal.target_name)
        except MoveItCommanderException:
            result.result = False
            self.action_plan_execute_named_pose.set_succeeded(result)
            return

        rospy.loginfo('Planning named [%s] pose...' % goal.target_name)
        plan1 = self.group.plan()

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.traj_publisher.publish(display_trajectory)

        rospy.sleep(0.5)

        rospy.loginfo('Start moving...')
        self.group.go(wait=True)

        rospy.loginfo('Planning named pose succeeded.')
        self.action_plan_execute_named_pose.set_succeeded(result)

    def plan_execute_pose_constraints_cb(self, goal):
        feedback = PlanExecutePoseConstraintsFeedback()
        result = PlanExecutePoseConstraintsResult()
        result.result = True


        self.contraints.name = "constraints"
        for js in goal.joint_constraints:
            self.contraints.joint_constraints.append(js)
        self.group.set_path_constraints(self.contraints)


        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()

        try:
            self.group.set_pose_target(goal.target_pose)
        except MoveItCommanderException:
            result.result = False
            return

        rospy.loginfo('Planning goal pose...')
        plan1 = self.group.plan()

        if len(plan1.joint_trajectory.points) == 0:
            result.result = False
            return

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.traj_publisher.publish(display_trajectory)

        rospy.sleep(0.5)

        rospy.loginfo('Start moving...')
        self.group.go(wait=True)
        rospy.sleep(2.0)
        self.group.set_path_constraints(None)
        self.contraints.joint_constraints = []

        rospy.loginfo('Planning goal pose succeeded.')
        self.action_plan_execute_pose_w_constraints.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('moveit_client_node', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    m = MoveitClientNode()

    rospy.spin()
