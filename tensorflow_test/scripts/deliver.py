#!/usr/bin/env python

import rospy
import tensorflow as tf
from std_msgs.msg import String
from std_msgs.msg import Float32

tf.Session()

def callback(data):
	z = sess.run(y, {x:data.data})
	rospy.loginfo(z)
	pub.publish(z)

def tensor():
    x = tf.placeholder(tf.float32)
    y = tf.add(x, 2.)
    sess = tf.Session()
    return sess, y, x

def deliver():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('deliver', anonymous=True)
#	with tf.Session() as sess:
	rospy.Subscriber('tester', Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	pub = rospy.Publisher('delivertopic', Float32, queue_size=10)
#	x = tf.placeholder(tf.float32)
#	y = tf.add(x, 2.)
#	sess = tf.Session()
	sess, y, x = tensor()
	deliver()
