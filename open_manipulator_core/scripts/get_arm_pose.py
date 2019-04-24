#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import JointState

def callback(jointState):
   pose_name = raw_input("name of pose: ")
   print "\n\n"
   print "    <group_state name=\"" + pose_name + "\" group=\"arm\">"
   for i in range(0, len(jointState.name) - 1):
      print "        <joint name=\"id_" + str(i + 1) + "\" value=\"" + str(jointState.position[i]) + "\" />"
   print "    </group_state>"

def listener():
   rospy.init_node('move_group_get_arm_pose', anonymous=True)
   rospy.Subscriber("joint_states", JointState, callback);
   rospy.spin()
   
if __name__ == '__main__':
   listener()
