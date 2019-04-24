#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_gripper_test', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
gripper_group = moveit_commander.MoveGroupCommander('gripper')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)



rospy.sleep(0.5)

group_variable_values = gripper_group.get_current_joint_values()
print "============ Joint values: ", group_variable_values


group_variable_values[2] = 2.11
gripper_group.set_joint_value_target(group_variable_values)

plan2 = gripper_group.plan()

print "============ Waiting while RVIZ displays plan2..."
rospy.sleep(2)

gripper_group.execute(plan2)
gripper_group.clear_pose_targets()
gripper_group.get_current_joint_values()

moveit_commander.roscpp_shutdown()
