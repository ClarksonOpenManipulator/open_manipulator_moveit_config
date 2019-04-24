#!/usr/bin/env python

import rospy
from dynamixel_workbench_msgs.srv import *

def torque_on_off(id, on_off):
	rospy.wait_for_service('position_controller/dynamixel_command')
	try:
		torque_service = rospy.ServiceProxy('position_controller/dynamixel_command', DynamixelCommand)
		resp = torque_service('', id, "Torque_Enable", on_off)
		return resp.comm_result
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
        
        
if __name__ == "__main__":
	option = int(raw_input("Torque on (1) or off (0): "))
	for i in range(1, 8):
		print torque_on_off(i, option)
