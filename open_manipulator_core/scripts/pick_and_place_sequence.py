#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Header
import geometry_msgs.msg


def move_to_named_pose(name):
   arm_group.clear_pose_targets()
   
   joint_state = sensor_msgs.msg.JointState()
   joint_state.header = Header()
   joint_state.header.stamp = rospy.Time.now()
   joint_state.position = arm_group.get_current_joint_values()
   moveit_robot_state = moveit_msgs.msg.RobotState()
   moveit_robot_state.joint_state = joint_state
   arm_group.set_start_state(moveit_robot_state)
   
   arm_group.set_named_target(name)
   plan = arm_group.plan()
   
   arm_group.execute(plan, wait=True)
   
def robot_grip(gripped):
   rospy.sleep(1)
   if gripped:
      grip_on_off_publisher.publish("grip_on")
   else:
      grip_on_off_publisher.publish("grip_off")
   rospy.sleep(1)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_pick_and_place', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

grip_on_off_publisher = rospy.Publisher('/gripper_command', std_msgs.msg.String, queue_size=1)

move_to_named_pose("start")
robot_grip(False)
move_to_named_pose("cube_apr")
move_to_named_pose("cube_pick")
robot_grip(True)
move_to_named_pose("cube_apr")
move_to_named_pose("place_apr")
move_to_named_pose("place_1")
robot_grip(False)
move_to_named_pose("place_apr")
move_to_named_pose("cyl_apr")
move_to_named_pose("cyl_pick")
robot_grip(True)
move_to_named_pose("cyl_apr")
move_to_named_pose("cyl_place_apr")
move_to_named_pose("cyl_place_apr_2")
move_to_named_pose("place_2")
robot_grip(False)
move_to_named_pose("cyl_place_apr_2")
move_to_named_pose("cyl_place_apr")
move_to_named_pose("place_apr")
move_to_named_pose("cir_apr")
move_to_named_pose("cir_pick")
robot_grip(True)
move_to_named_pose("cir_apr")
move_to_named_pose("cir_place_apr")
move_to_named_pose("cir_place_apr_2")
move_to_named_pose("place_3")
robot_grip(False)
move_to_named_pose("cir_place_apr_2")
move_to_named_pose("place_apr")
move_to_named_pose("start")

moveit_commander.roscpp_shutdown()
