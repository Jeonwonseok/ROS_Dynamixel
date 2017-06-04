#!/usr/bin/env python

import rospy
import numpy as np
from beginner_tutorials.msg import Data

def talker():
	pub = rospy.Publisher('tester2', Data, queue_size=10)
	rospy.init_node('talker2', anonymous=True)
	rate = rospy.Rate(1)
	data = Data()
	data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	i = 2048
	k = 100
	while not rospy.is_shutdown():
		i = i + k		
		if i > 2500:
			k = -1*k
		elif i < 1500:
			k = -1*k
		for j in xrange(12):
			data[j] = i+50*j
		rospy.loginfo(data)
		pub.publish(data)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
