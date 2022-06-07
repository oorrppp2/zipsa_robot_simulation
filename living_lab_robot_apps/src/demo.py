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
from behaviors.utils import *

from living_lab_robot_moveit_client.msg import PlanExecuteNamedPoseAction, PlanExecuteNamedPoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseAction, PlanExecutePoseGoal
from living_lab_robot_moveit_client.msg import PlanExecutePoseConstraintsAction, PlanExecutePoseConstraintsGoal
from living_lab_robot_perception.msg import ObjectDetectAction, ObjectDetectGoal
from living_lab_robot_perception.msg import ReceiveTargetAction, ReceiveTargetGoal

global Point_data
global Point_flag

def create_root():

    root = py_trees.composites.Parallel("demo")
    done_scene = DonePlayScene(name="done_scene")

    gripper_close = MoveJoint(name="gripper_close", controller_name="/gripper_controller", command=0.0)
    gripper_open = MoveJoint(name="gripper_open", controller_name="/gripper_controller", command=1.0)

    wait_time1 = WaitForTime(name="delay_1s", time=1.0)
    wait_time05 = WaitForTime(name="delay_0.5s", time=0.5)

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

    shake_arm_left = PlanExecuteNamedPoseGoal()
    shake_arm_left.target_name ="shake_arm_left"
    move_manipulator_to_left = py_trees_ros.actions.ActionClient(
        name="shake_arm_left",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=shake_arm_left
    )
    
    shake_arm_right = PlanExecuteNamedPoseGoal()
    shake_arm_right.target_name ="shake_arm_right"
    move_manipulator_to_right = py_trees_ros.actions.ActionClient(
        name="shake_arm_right",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=shake_arm_right
    )

    pour_juice_ready = PlanExecuteNamedPoseGoal()
    pour_juice_ready.target_name ="pour_juice_ready"
    move_manipulator_to_pour_juice_ready = py_trees_ros.actions.ActionClient(
        name="pour_juice_ready",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=pour_juice_ready
    )

    pour_juice_into_cup = PlanExecuteNamedPoseGoal()
    pour_juice_into_cup.target_name ="pour_juice_into_cup"
    move_manipulator_to_pour_juice_into_cup = py_trees_ros.actions.ActionClient(
        name="pour_juice_into_cup",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=pour_juice_into_cup
    )

    grasp_bowl_ready = PlanExecuteNamedPoseGoal()
    grasp_bowl_ready.target_name ="grasp_bowl_ready"
    move_manipulator_to_grasp_bowl_ready = py_trees_ros.actions.ActionClient(
        name="grasp_bowl_ready",
        action_namespace="/plan_and_execute_named_pose",
        action_spec=PlanExecuteNamedPoseAction,
        action_goal=grasp_bowl_ready
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

    # Shaking arm
    shake_arm = py_trees.composites.Sequence("shake_arm")

    wait_shake_arm = py_trees_ros.subscribers.CheckData(name="wait_shake_arm", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="shake_arm")

    shake_arm.add_children(
        [wait_shake_arm,
         move_manipulator_to_left,
         move_manipulator_to_right,
         move_manipulator_to_left,
         move_manipulator_to_right,
         move_manipulator_to_left,
         move_manipulator_to_right,
         ]
    )

    # Pour juice into a cup
    pour_juice = py_trees.composites.Sequence("pour_juice")

    wait_pour_juice = py_trees_ros.subscribers.CheckData(name="wait_pour_juice", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="pour_juice")

    pour_juice.add_children(
        [wait_pour_juice,
         move_manipulator_to_pour_juice_ready,
        #  wait_time1,
         move_manipulator_to_pour_juice_into_cup,
         move_manipulator_to_pour_juice_ready,
         ]
    )

    # Grasp bowl
    grasp_bowl_ready = py_trees.composites.Sequence("grasp_bowl_ready")

    wait_grasp_bowl_ready = py_trees_ros.subscribers.CheckData(name="wait_grasp_bowl_ready", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="grasp_bowl_ready")

    move_manipulator_to_grasp_bowl = GraspBowlActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=0.0,
        y_offset=0.0,
        z_offset=-0.13,
        constraint=True,
        joint={'body_rotate_joint':[0.0, 5 * math.pi / 180.0, 5 * math.pi / 180.0],}
    )

    grasp_bowl_ready.add_children(
        [wait_grasp_bowl_ready,
         move_manipulator_to_grasp_bowl_ready,
         move_manipulator_to_grasp_bowl,
         ]
    )


    #
    # intro  (Introduce and choose the object.)
    #
    intro = py_trees.composites.Sequence("intro")

    wait_intro = py_trees_ros.subscribers.CheckData(name="wait_intro_demo", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="intro")

#    start_scene1 = Print_message(name="* Move to ready pose *")
    start_mention1 = Print_message(name="* Introduce *")

    # scene1_say1 = Say(name="say_hello1", text='안녕하세요? 저는 리빙랩의 로봇 집사, 입니다.')
    """
        적절한 멘트
    """

    order_mention1 = Print_message(name="Choose cup / cup / milk")
    # scene1_say2 = Say(name="say_request1", text='컵, 사과, 우류 중 어떤것을 가져다 드릴까요?')
    order_object = OrderActionClient(
        name="order_target",
        action_namespace="/order_received",
        action_spec=ReceiveTargetAction,
        action_goal=ReceiveTargetGoal()
    )

    # order_object = OrderActionClient(
    #     name="order_received",
    #     action_namespace="/sst_order_received",
    #     action_spec=ReceiveTargetAction,
    #     action_goal=ReceiveTargetGoal()
    # )
    arm_pull_out = Fold_arm("Pull out", 0)
    done_scene_1 = Publish(topic_name="/wait_done_scene", data="scene_1_done")

    intro.add_children(
        [wait_intro,
         start_mention1,
         order_mention1,
         # scene1_say2,
         order_object,
         # scene1_say2,
         done_scene_1,
         ]
    )


    move_to_table = py_trees.composites.Sequence("move_to_table")

    wait_move_to_table = py_trees_ros.subscribers.CheckData(name="wait_move_to_table", topic_name="/wait_select_scene", topic_type=String,
            variable_name="data", expected_value="move_to_table")

    move_to_table_mention1 = Print_message(name="* Move_to_table *")
    # coffee table
    goal_table = move_base_msgs.msg.MoveBaseGoal()
    goal_table.target_pose.header.frame_id = "map"
    goal_table.target_pose.header.stamp = rospy.Time.now()

    # kitchen table
    goal_table.target_pose.pose.position.x = 1.638
    goal_table.target_pose.pose.position.y = -1.716

    goal_table.target_pose.pose.orientation.x = 0
    goal_table.target_pose.pose.orientation.y = 0
    goal_table.target_pose.pose.orientation.z = 0
    goal_table.target_pose.pose.orientation.w = 1.0

    move_to_table_action = py_trees_ros.actions.ActionClient(
        name="move to table",
        action_namespace="/move_base",
        action_spec=move_base_msgs.msg.MoveBaseAction,
        action_goal=goal_table
    )

    move_to_table.add_children(
        [wait_move_to_table,
        move_to_table_mention1,
        move_to_table_action,
        arm_pull_out,
        head_tilt_down,
        done_scene,
        ]
    )

    
    # Find_target  (Find object.)
    

    find_target = py_trees.composites.Sequence("find_target")

    wait_find_target= py_trees_ros.subscribers.CheckData(name="wait_find_target", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="find_target")

#    start_scene3 = Print_message(name="* Scene  *")
    find_target_mention1 = Print_message(name="Finding target ...")

    find_object = ObjectDetectionActionClient(
        name="find_object",
        action_namespace="/object_detect",
        action_spec=ObjectDetectAction,
        action_goal=ObjectDetectGoal()
    )
    done_scene_2 = Publish(topic_name="/wait_done_scene", data="scene_2_done")

    find_target.add_children(
        [wait_find_target,
         find_target_mention1,
         publish_resume_request,
         find_object,
        #  publish_resume_request,
        #  find_object,
         done_scene_2,
         ]
    )

    #
    # Arm_control  (Move arm to the target object to grasp it.)
    #

    arm_control = py_trees.composites.Sequence("arm_control")
    wait_arm_control = py_trees_ros.subscribers.CheckData(name="wait_arm_control", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="arm_control")
    arm_control_mention1 = Print_message(name="* Arm_control *")

    move_manipulator_to_grasp_add_offset = GraspActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=-0.05,
        y_offset=0.05,
        z_offset=0.05,
        constraint=True,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[-0.05, 0.0, 0.35]}
#        joint=["arm1_joint", "arm6_joint"]
    )
    move_manipulator_to_grasp = GraspActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        x_offset=0.03,
        y_offset=0.05,
        z_offset=-0.01,
        constraint=True,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[-0.05, 0.0, 0.35]}
    )
    done_scene_3 = Publish(topic_name="/wait_done_scene", data="scene_3_done")

    arm_control.add_children(
        [wait_arm_control,
         arm_control_mention1,
         publish_pause_request,
         # find_object,
         move_manipulator_to_grasp_add_offset,
         wait_time1,
         wait_time1,
         move_manipulator_to_grasp,
         #move_manipulator_to_grasp_ready,
         done_scene_3,
         ]
    )

    #
    # Grasp the object.  (Gripper close and arm position into 'grasp_done')
    #
    grasp_object = py_trees.composites.Sequence("grasp_object")

    wait_grasp_object = py_trees_ros.subscribers.CheckData(name="wait_grasp_object", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="grasp_object")

    grasp_object_mention1 = Print_message(name="* Closing the gripper *")

    elevation_up_action = Elevation_up(target_pose=0.05)
    done_scene_4 = Publish(topic_name="/wait_done_scene", data="scene_4_done")

    grasp_object.add_children(
        [wait_grasp_object,
         grasp_object_mention1,
         gripper_close,
         wait_time1,
         elevation_up_action,
         wait_time1,
         move_manipulator_to_grasp_done,
         publish_resume_request,
         done_scene_4,
         ]
    )

    #
    # Put the object down. 
    #
    put_object = py_trees.composites.Sequence("put_object")

    wait_put_object = py_trees_ros.subscribers.CheckData(name="wait_put_object", topic_name="/wait_select_scene", topic_type=String,
        variable_name="data", expected_value="put_object")

    put_object_mention1 = Print_message(name="* Putting down the object*")

    move_manipulator_to_put_down = GraspActionClient(
        name="move_manipulator_to_grasp",
        action_namespace="/plan_and_execute_pose_w_joint_constraints",
        action_spec=PlanExecutePoseConstraintsAction,
        action_goal=PlanExecutePoseConstraintsGoal(),
        constraint=True,
        x_offset=0,
        joint={'arm1_joint':[0.0, 30 * math.pi / 180.0, 30 * math.pi / 180.0],
			'arm4_joint':[0.0, 90 * math.pi / 180.0, 90 * math.pi / 180.0],
			'arm6_joint':[0.0, 10 * math.pi / 180.0, 10 * math.pi / 180.0],
			'elevation_joint':[-0.05, 0.0, 0.35]},
        mode="put"
    )
    elevation_down_action = Elevation_up(target_pose=-0.07)
    done_scene_5 = Publish(topic_name="/wait_done_scene", data="scene_5_done")

    put_object.add_children(
        [wait_put_object,
         put_object_mention1,
         move_manipulator_to_put_down,
         wait_time1,
         wait_time1,
         elevation_down_action,
         wait_time1,
         wait_time1,
         gripper_open,
         move_manipulator_to_home,
         done_scene_5,
         ]
    )

    finish_demo = py_trees.composites.Sequence("finish_demo")
    wait_finish_demo = py_trees_ros.subscribers.CheckData(name="wait_finish_demo", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="finish_demo")
    finish_demo_mention1 = Print_message(name="* Finish_demo *")

    finish_demo.add_children(
        [wait_finish_demo,
         finish_demo_mention1,
         head_tilt_down,
         move_manipulator_to_home,
         arm_put_in,
         done_scene,
         ]
    )

    elevation_up = py_trees.composites.Sequence("Elevation_up")
    wait_elevation_up = py_trees_ros.subscribers.CheckData(name="wait_elevation_up", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="elevation_up")
    elevation_up_mention1 = Print_message(name="* Elevation_up *")

#			Elevation_up : desired elevation position = current position + target_pose
    elevation_up_10cm_action = Elevation_up(target_pose=0.1)
    elevation_down_10cm_action = Elevation_up(target_pose=-0.1)

    elevation_up.add_children(
        [wait_elevation_up,
         elevation_up_mention1,
         elevation_up_10cm_action,
         done_scene,
         ]
    )

    elevation_down = py_trees.composites.Sequence("Elevation_down")
    wait_elevation_down = py_trees_ros.subscribers.CheckData(name="wait_elevation_down", topic_name="/wait_select_scene", topic_type=String,
           variable_name="data", expected_value="elevation_down")
    elevation_down_mention1 = Print_message(name="* Elevation_down *")

#			Elevation_up : desired elevation position = current position + target_pose

    elevation_down.add_children(
        [wait_elevation_down,
         elevation_down_mention1,
         elevation_down_10cm_action,
         done_scene,
         ]
    )

    root.add_children([grasp_bowl_ready, pour_juice, shake_arm, gripper_open_cmd, intro, move_to_table, find_target, arm_control, grasp_object, finish_demo, put_object, elevation_up, elevation_down])
    # root.add_children([intro])
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
