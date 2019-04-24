#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg
import sensor_msgs.msg

def print_list(the_list):
   for i in the_list:
      print i,

def joint_state_callback(data):
   print_list(data.position)
   #print_list(data.velocity)
   #print_list(data.effort)
   print data.header.stamp

def planned_path_callback(data):
   for i in data.trajectory[0].joint_trajectory.points:
      print_list(i.positions)
      print_list(i.velocities)
      print_list(i.accelerations)
      print i.time_from_start

def joint_state_listener():
   rospy.init_node('record_joint_states')
   rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, joint_state_callback)
   rospy.spin()   
   
def trajectory_listener():
   rospy.init_node('record_trajectory_plan')
   rospy.Subscriber("move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, planned_path_callback)
   rospy.spin()
   

   

mode = raw_input("Choose mode (j)oint states or (t)rajectory: ")
if (mode == "j"):
   joint_state_listener()
elif (mode == "t"):
   trajectory_listener()
