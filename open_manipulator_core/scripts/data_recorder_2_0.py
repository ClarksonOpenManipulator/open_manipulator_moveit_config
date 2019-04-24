#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg
import sensor_msgs.msg
import open_manipulator_core.msg


class Recorder:
   def __init__(self, file_name):
      self.data_file = open(file_name, "w")
         
   def start_recorder(self,mode):
      if (mode == "j"):
         self.joint_state_listener()
      elif (mode == "t"):
         self.trajectory_listener()
      elif (mode == "d"):
         self.debug_listener()
         
   def joint_state_listener(self):
      rospy.init_node('record_joint_states')
      rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)
      rospy.spin()
   
   def trajectory_listener(self):
      rospy.init_node('record_trajectory_plan')
      rospy.Subscriber("move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, self.planned_path_callback)
      rospy.spin()
      
   def debug_listener(self):
      rospy.init_node('record_debug')
      rospy.Subscriber("debug_data", open_manipulator_core.msg.DynamixelDebug, self.record_data_callback)
      rospy.spin()
      
   def planned_path_callback(self, data):
      for i in data.trajectory[0].joint_trajectory.points:
         self.print_list(i.positions)
         self.print_list(i.velocities)
         self.print_list(i.accelerations)
         self.write_data(i.time_from_start)
         self.write_data("\n")
         self.write_data("\n")
         
   def joint_state_callback(self, data):
      self.print_list(data.position)
      #print_list(data.velocity)
      #print_list(data.effort)
      self.write_data(data.header.stamp)
      self.write_data("\n")
      
   def record_data_callback(self, data):
      self.print_list(data.dxl_id)
      self.print_list(data.present_temp)
      self.print_list(data.present_load)
      self.print_list(data.present_volt)
      self.print_list(data.present_current)
      self.print_list(data.present_pos)
      self.print_list(data.goal_pos)

   def print_list(self, the_list):
      for i in the_list:
         self.write_data(i)
         
   def write_data(self, data):
      self.data_file.write(str(data) + " ")


mode = raw_input("Choose mode (j)oint states, (t)rajectory or (d)ebug: ")
file_name = raw_input("Enter name for output file (if file already exists it will be overwriten): ")
recorder = Recorder(file_name)
recorder.start_recorder(mode)
