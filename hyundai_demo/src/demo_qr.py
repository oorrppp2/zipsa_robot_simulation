#!/usr/bin/python
#-*- encoding: utf8 -*-

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

from std_msgs.msg import Empty, String, Bool, Header
from geometry_msgs.msg import PointStamped
import move_base_msgs
from tf.transformations import quaternion_from_euler

from behaviors.speech import *
from behaviors.move_arm_controller import *
from behaviors.move_joint import *
from behaviors.lamp_control import *
from behaviors.wait_time import *
from behaviors.actions import *
from behaviors.gaze_sync_control import *
from behaviors.app_control import *
from behaviors.print_state import *

from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_perception.msg import QRCodeDetectAction, QRCodeDetectGoal

global Point_data
global Point_flag
Point_data = PointStamped()
Point_flag = False

def create_root():

    root = py_trees.composites.Parallel("demo")
    done_scene = DonePlayScene(name="done_scene")
    #
    # Scene #1
    #

    scene1 = py_trees.composites.Sequence("scene6_grasp")

    wait_scene6_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="scene1")

    find_qr_code = QRCodeActionClient(
        name="find_qr_code",
        action_namespace="/qrcode_detect",
        action_spec=QRCodeDetectAction,
        action_goal=QRCodeDetectGoal()
    )

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

    move_manipulator_to_grasp1 = GraspActionClient(
        name="move_manipulator_to_grasp1",
        action_namespace="/plan_and_execute_pose",
        action_spec=PlanExecutePoseAction,
        action_goal=PlanExecutePoseGoal()
    )


    scene1.add_children(
        [ wait_scene6_intro,
          find_qr_code,
          move_manipulator_to_grasp_ready,
          move_manipulator_to_grasp1,
          move_manipulator_to_grasp_done,
          done_scene
        ]
    )



    root.add_children([scene1])
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
