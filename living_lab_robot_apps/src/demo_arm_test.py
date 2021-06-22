#!/usr/bin/python
#-*- encoding: utf8 -*-

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import tf2_ros

from std_msgs.msg import Empty, String, Bool, Header
from geometry_msgs.msg import PointStamped
import move_base_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

from behaviors.speech import *
from behaviors.move_arm_controller import *
from behaviors.move_joint import *
from behaviors.lamp_control import *
from behaviors.wait_time import *
from behaviors.actions_test import *
from behaviors.gaze_sync_control import *
from behaviors.app_control import *
from behaviors.utils import *

from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from living_lab_robot_perception.msg import ObjectDetectAction, ObjectDetectGoal
from living_lab_robot_perception.msg import ReceiveTargetAction, ReceiveTargetGoal

global Point_data
global Point_flag

x_offset = 0.8
y_offset = 0.5
z_offset = 0.7

def create_root():

    root = py_trees.composites.Parallel("demo")
    done_scene = DonePlayScene(name="done_scene")

    gripper_close = MoveJoint(name="gripper_close", controller_name="/gripper_controller", command=0.0)
    gripper_open = MoveJoint(name="gripper_open", controller_name="/gripper_controller", command=1.0)

    goal_grap_ready = PlanExecuteNamedPoseGoal()
    goal_grap_ready.target_name ="grasp_ready"
    move_manipulator_to_grasp_ready = py_trees_ros.actions.ActionClient(
        name="move_manipulator_to_grasp_ready",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=goal_grap_ready
    )

    goal_grap_done = PlanExecuteNamedPoseGoal()
    goal_grap_done.target_name ="grasp_done"
    move_manipulator_to_grasp_done = py_trees_ros.actions.ActionClient(
        name="move_manipulator_to_grasp_done",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=goal_grap_done
    )

    goal_home = PlanExecuteNamedPoseGoal()
    goal_home.target_name ="home"
    move_manipulator_to_home = py_trees_ros.actions.ActionClient(
        name="move_manipulator_to_home",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=goal_home
    )

    head_tilt_up = MoveJoint(name="tilt_up", controller_name="/tilt_controller", command=0.0)
    head_tilt_down = MoveJoint(name="tilt_down", controller_name="/tilt_controller", command=0.4)


    publish_pause_request = Publish(topic_name="/pause_request", data="pause")
    publish_resume_request = Publish(topic_name="/pause_request", data="resume")

    #
    # gripper_open  (gripper_open arm)
    #
    gripper_open_cmd = py_trees.composites.Sequence("gripper_open")

    wait_gripper_open = py_trees_ros.subscribers.CheckData(name="wait_gripper_open", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="gripper_open")

    start_gripper_open = Print_message(name="* Opening the gripper *")

    gripper_open_cmd.add_children(
        [wait_gripper_open,
         start_gripper_open,
         gripper_open,
         done_scene,
         ]
    )

    #
    # fold_arm  (fold arm)
    #
    fold_arm = py_trees.composites.Sequence("fold_arm")

    wait_fold_arm = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="fold_arm")

    start_fold_arm = Print_message(name="* Folding the arm *")
    arm_put_in = Fold_arm("Put in", -0.15)

    fold_arm.add_children(
        [wait_fold_arm,
         start_fold_arm,
         move_manipulator_to_home,
         arm_put_in,
         done_scene,
         ]
    )


    #
    # Arm_control  (Move arm to the target object to grasp it.)
    #

    arm_control = py_trees.composites.Sequence("arm_control")
    wait_arm_control = py_trees_ros.subscribers.CheckData(name="wait_arm_control", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="arm_control")
    arm_control_mention1 = Print_message(name="* Arm_control *")

    body_rotate_05 = Body_Rotate(
        x_offset=x_offset,
        y_offset=y_offset,
        z_offset=z_offset
        )

    move_manipulator = GraspActionClient(
        name="move_manipulator",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=x_offset,
        y_offset=y_offset,
        z_offset=z_offset,
        constraint=True,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[-0.05, 0.0, 0.35]
			# 'body_rotate_joint':[0.0, 0.0, 0.0]
            }
#        joint=["arm1_joint", "arm6_joint"]
    )

    arm_control.add_children(
        [wait_arm_control,
         arm_control_mention1,
         body_rotate_05,
         move_manipulator,
         ]
    )

    arm_control2 = py_trees.composites.Sequence("arm_control2")
    wait_arm_control2 = py_trees_ros.subscribers.CheckData(name="wait_arm_control2", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="arm_control2")
    arm_control_mention2 = Print_message(name="* Arm_control2 *")

    body_rotate_2 = Body_Rotate(
        x_offset=x_offset,
        y_offset=-y_offset,
        z_offset=z_offset
        )

    move_manipulator2 = GraspActionClient(
        name="move_manipulator",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=x_offset,
        y_offset=-y_offset,
        z_offset=z_offset,
        constraint=True,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[-0.05, 0.0, 0.35]
			# 'body_rotate_joint':[0.0, 0.0, 0.0]
            }
#        joint=["arm1_joint", "arm6_joint"]
    )

    arm_control2.add_children(
        [wait_arm_control2,
         arm_control_mention2,
         body_rotate_2,
         move_manipulator2,
         ]
    )
    #
    # Body_rotate
    #

    body_rotate = py_trees.composites.Sequence("body_rotate")
    wait_body_rotate = py_trees_ros.subscribers.CheckData(name="wait_body_rotate", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="body_rotate")
    body_rotate_mention1 = Print_message(name="* Body_rotate *")

    # body_rotate_05 = Body_Rotate(target_pose=0.5)

    body_rotate.add_children(
        [wait_body_rotate,
         body_rotate_mention1,
        #  body_rotate_05,
         ]
    )
    root.add_children([arm_control, body_rotate, arm_control2])
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__ == '__main__':
    rospy.init_node('app_introduction', anonymous=False)
    #py_trees.logging.level = py_trees.logging.Level.DEBUG

    print("starting..")

    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    print('initialize...')
    behaviour_tree.tick_tock(500)

    rospy.spin()
