#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Header
import geometry_msgs.msg
import math

def circle(radius,res,start):
   waypoints = []
   waypoints.append(start)
   for i in range(0,res):
      point = copy.deepcopy(start)
      point.position.x += radius*math.cos((2*math.pi/res)*i)
      point.position.y += radius*math.sin((2*math.pi/res)*i)
      point.position.z =.2
      waypoints.append(copy.deepcopy(point))
   waypoints.append(start)
   return waypoints                

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_circle', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


r = .075
#t = 5
#v = 3.524
#s = 2 * v * math.pi * (r / t)

path_points = circle(r, 100, arm_group.get_current_pose().pose)
#for i in path_points:
 #  print "x: " + str(i.position.x) + " y: " + str(i.position.y) + " z: " + str(i.position.z) 

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:

(plan, fraction) = arm_group.compute_cartesian_path(
                                   path_points,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

plan = arm_group.retime_trajectory(robot.get_current_state(), plan, .05)  # (0.02) time scaling factor

# Note: We are just planning, not asking move_group to actually move the robot yet:
#return plan, fraction
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory);

arm_group.execute(plan, wait=True)
# Excutes trajectory

moveit_commander.roscpp_shutdown()
