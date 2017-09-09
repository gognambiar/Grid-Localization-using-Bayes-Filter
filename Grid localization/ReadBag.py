#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as np
from std_msgs.msg import Int32, String
roslib.load_manifest('lab4')

def readbag():
	bag = rosbag.Bag('grid.bag')
	try:
		for tp, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
			if tp == 'Movements':
				print tp
                                print msg
			else: 
				print tp
                                print msg
				break

	finally:
		bag.close()

if __name__ == '__main__':
	rospy.init_node('lab4', anonymous=True)
	readbag()



