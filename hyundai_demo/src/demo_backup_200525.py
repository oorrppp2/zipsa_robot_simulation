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
from behaviors.actions import *
from behaviors.gaze_sync_control import *
from behaviors.app_control import *
from behaviors.print_state import *

from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from living_lab_robot_perception.msg import QRCodeDetectAction, QRCodeDetectGoal

global Point_data
global Point_flag

def create_root():

    root = py_trees.composites.Parallel("demo")
    done_scene = DonePlayScene(name="done_scene")

    gripper_close = MoveJoint(name="gripper_close", controller_name="/gripper_controller", command=0.5)
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

    target_x = 0.7
    target_y = -0.3
    target_z = 0.8


    #
    # init joint
    #
    init_joint = py_trees.composites.Sequence("init_joint")

    wait_init_joint = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="init")

    start_init_joint = PrintState_Behaviour(name="* Initiate the joints *")
    init_joint_1 = Constraint_joint("init", 0.5)

    init_joint.add_children(
        [wait_init_joint,
         start_init_joint,
         init_joint_1,
         done_scene,
         ]
    )

    #
    # gripper_close  (gripper_close arm)
    #
    gripper_close_cmd = py_trees.composites.Sequence("gripper_close")

    wait_gripper_close = py_trees_ros.subscribers.CheckData(name="wait_gripper_close", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="gripper_close")

    start_gripper_close = PrintState_Behaviour(name="* Closing the gripper *")

    gripper_close_cmd.add_children(
        [wait_gripper_close,
         start_gripper_close,
         gripper_close,
         done_scene,
         ]
    )

    #
    # gripper_open  (gripper_open arm)
    #
    gripper_open_cmd = py_trees.composites.Sequence("gripper_open")

    wait_gripper_open = py_trees_ros.subscribers.CheckData(name="wait_gripper_open", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="gripper_open")

    start_gripper_open = PrintState_Behaviour(name="* Opening the gripper *")

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

    start_fold_arm = PrintState_Behaviour(name="* Folding the arm *")
    arm_folding = Fold_arm("Put in", -0.15)

    fold_arm.add_children(
        [wait_fold_arm,
         start_fold_arm,
         move_manipulator_to_home,
         arm_folding,
         done_scene,
         ]
    )

    #
    # Scene #1  (Introduce and choose the object.)
    #
    scene1 = py_trees.composites.Sequence("scene1_intro")

    wait_scene1_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="scene1")

    # start_scene1 = PrintState_Behaviour(name="* Scene1 *")
    start_scene1 = PrintState_Behaviour(name="* Move to ready pose *")
    # scene1_say1 = PrintState_Behaviour(name="Hi! I'm ZipSa.")

    # scene1_say1 = Say(name="say_hello1", text='안녕하세요? 저는 리빙랩의 로봇 집사, 입니다.')
    """
        적절한 멘트
    """

    scene1_say2 = PrintState_Behaviour(name="Choose cup / apple / milk")
    # scene1_say2 = Say(name="say_request1", text='컵, 사과, 우류 중 어떤것을 가져다 드릴까요?')
    find_object = OrderActionClient(
        name="order_target",
        action_namespace="/order_received",
        action_spec=QRCodeDetectAction,
        action_goal=QRCodeDetectGoal()
    )

    arm_pull_out = Fold_arm("Pull out", 0)

    scene1.add_children(
        [wait_scene1_intro,
         start_scene1,
         #scene1_say1,
         # scene1_say2,
         #find_object,
         # scene1_say2,
         # arm_pull_out,
         # move_manipulator_to_grasp_ready,
         move_manipulator_to_home,
         done_scene,
         ]
    )

    #
    # Choose object cup or apple or milk
    #
    # scene1 = py_trees.composites.Sequence("scene1_intro")
    #
    # wait_scene1_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
    #     variable_name="data", expected_value="intro")

    head_tilt_up = MoveJoint(name="tilt_down", controller_name="/tilt_controller", command=0.0)

    #
    # Scene #3  (Find object and positioning the arm.)
    #

    scene3 = py_trees.composites.Sequence("find_object")

    wait_scene3_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="scene3")

    start_scene3 = PrintState_Behaviour(name="* Scene3 *")
    scene3_say1 = PrintState_Behaviour(name="Finding ...")
    wait_time1 = WaitForTime(name="delay_1s", time=1.0)

    find_object = QRCodeActionClient(
        name="find_qr_code",
        action_namespace="/qrcode_detect",
        action_spec=QRCodeDetectAction,
        action_goal=QRCodeDetectGoal()
    )

    move_manipulator_to_grasp1 = GraspActionClient(
        name="move_manipulator_to_grasp1",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=target_x-0.08,
        y_offset=target_y,
        z_offset=target_z+0.05,
        constraint=True,
        joint="arm1_joint"
    )
    move_manipulator_to_grasp2 = GraspActionClient(
        name="move_manipulator_to_grasp2",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=target_x,
        y_offset=target_y,
        z_offset=target_z,
        constraint=True,
        joint="arm1_joint"
    )

    scene3.add_children(
        [wait_scene3_intro,
         start_scene3,
         scene3_say1,
         # find_object,
         move_manipulator_to_grasp1,
         move_manipulator_to_grasp2,
         done_scene,
         ]
    )

    scene4 = py_trees.composites.Sequence("find_object")
    wait_scene4_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="scene4")
    start_scene4 = PrintState_Behaviour(name="* Scene4 *")

    move_manipulator_to_grasp3 = GraspActionClient(
        name="move_manipulator_to_grasp3",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=target_x-0.04,
        y_offset=target_y,
        z_offset=target_z+0.15,
        constraint=True,
        joint="arm1_joint"
    )

    scene4.add_children(
        [wait_scene4_intro,
         start_scene4,
         # find_object,
         move_manipulator_to_grasp3,
         move_manipulator_to_grasp_done,
         done_scene,
         ]
    )

    scene5 = py_trees.composites.Sequence("find_object")
    wait_scene5_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="scene5")
    start_scene5 = PrintState_Behaviour(name="* Scene5 *")
    # start_scene5 = PrintState_Behaviour(name="* Move to (0.77, 0.0, 0.95 *")
    find_object = QRCodeActionClient(
        name="find_qr_code",
        action_namespace="/qrcode_detect",
        action_spec=QRCodeDetectAction,
        action_goal=QRCodeDetectGoal()
    )
    scene5.add_children(
        [wait_scene5_intro,
         start_scene5,
#         find_object,
#          move_manipulator_to_grasp3,
         done_scene,
         ]
    )

    scene6 = py_trees.composites.Sequence("find_object")
    wait_scene6_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="scene6")
    start_scene6 = PrintState_Behaviour(name="* Scene6 *")
    find_object = QRCodeActionClient(
        name="find_qr_code",
        action_namespace="/qrcode_detect",
        action_spec=QRCodeDetectAction,
        action_goal=QRCodeDetectGoal()
    )
    move_manipulator_to_grasp6 = GraspActionClient(
        name="move_manipulator_to_grasp6",
        action_namespace="/plan_and_execute_pose",
        action_spec=PlanExecutePoseAction,
        action_goal=PlanExecutePoseGoal(),
        x_offset=0.6,
        y_offset=0.7,
        z_offset=0.45
    )
    scene6.add_children(
        [wait_scene6_intro,
         start_scene6,
         # gripper_close,
         # find_object,
         move_manipulator_to_grasp6,
         done_scene,
         ]
    )


    root.add_children([init_joint, gripper_close_cmd, gripper_open_cmd, scene3, scene4, scene5, scene6])
    # root.add_children([scene1, scene3, scene4, scene5, scene6, scene7])
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
