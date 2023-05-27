#!/usr/bin/env python3

# main.py
# Gavin, Shariar, Amesh
# May 25, 2023
# Main controller for VandalBot

# Also, remember to have serial_node.py running,
# which allows ROS to reach the Arduino over serial: 
# rosrun rosserial_python serial_node.py /dev/ttyASM0 

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

def talker():
	pub = rospy.Publisher('vandalbot/base', Quaternion, queue_size=10)
	rospy.init_node('vandalbot_brain', anonymous=False)
	rate = rospy.Rate(10)	 # Publish 10 times a second
	while not rospy.is_shutdown(): # handles interrupts
		pub.publish(1, 0, 0, 100)
		rospy.sleep(3)
		pub.publish(-1, 0, 0, 100)
		rospy.sleep(3)
		pub.publish(0, 1, 0, 100)
		rospy.sleep(3)
		pub.publish(0, -1, 0, 100)
		rospy.sleep(3)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
