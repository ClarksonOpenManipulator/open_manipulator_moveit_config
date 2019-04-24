#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_get_arm_pose', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)



rospy.sleep(5)

pick1 = arm_group.get_current_pose()
print type(pick1)
print pick1
pick1.pose.position.x = -0.00177646988389
pick1.pose.position.y = -0.0236930622291
pick1.pose.position.z = 0.443955522298
pick1.pose.orientation.x = 0.461532491145
pick1.pose.orientation.y = 0.583866638866
pick1.pose.orientation.z = 0.555570851315
pick1.pose.orientation.w = 0.37071355088

arm_group.set_pose_target(pick1)

plan1 = arm_group.plan()

print "============ Waiting while RVIZ displays plan..."
rospy.sleep(1)

print "============ Visualizing plan"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);

print "============ Waiting while plan is visualized (again)..."
rospy.sleep(1)

arm_group.execute(plan1)
arm_group.clear_pose_targets()
arm_group.get_current_joint_values()

moveit_commander.roscpp_shutdown()
