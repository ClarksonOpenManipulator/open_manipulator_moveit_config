#!/usr/bin/env python
import rospy
import signal
from sensor_msgs.msg import *
from std_msgs.msg import *

rospy.init_node("array_reader")
pub = rospy.Publisher("goal_dynamixel_position", JointState, queue_size = 1)

file_name = raw_input("File name: ")
hzs = raw_input("Clock hzs: ")
repeat = True if raw_input('Repeat? ("y" for yes): ') == "y" else False
print str(repeat)
rate = rospy.Rate(int(hzs))

state = rospy.wait_for_message("joint_states", JointState)
state.header = Header();

def signal_handler(sig, frame):
   global repeat
   print('\nLast loop...')
   repeat = False
signal.signal(signal.SIGINT, signal_handler)

data = []
with open(file_name) as file:
   for line in file:
      for num in line.split():
         data.append(float(num))
         

grip = state.position[6]
idx = 0
data_len = len(data)
while idx < data_len:
   point = []
   for i in range(0, 6):
      point.append(data[idx])
      idx += 1
   point.append(grip)
   state.position = point
   pub.publish(state)
   rate.sleep()
   if (idx >= data_len and repeat):
      idx = 0
