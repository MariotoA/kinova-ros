#! /usr/bin/env python

import rospy
import tf
import numpy as np
from pose_action_client import moveArm, currentCartesianCommand,Quaternion2EulerXYZ
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from kinova_msgs.srv import HomeArm,Start,Stop
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

HOME_STR = 'home_arm'
STOP_STR = 'stop'
START_STR = 'start'

YLIM = 0.1
ZLIM = 0

def callService(service_str,msg):
	address = "/j2n6s300_driver/in/{}".format(service_str)
	rospy.wait_for_service(address)
	try:
		homeArm = rospy.ServiceProxy(address, msg)
		return homeArm()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def validate(frame, trans):
	global YLIM
	global ZLIM
	y,z = trans[1:]
	# To not cause problems with the firmware we introduce waits
	if y >= YLIM or z < ZLIM:		
		print 'Error in frame {}: {}'.format(frame, trans)
		callService(STOP_STR,Stop)
		rospy.sleep(1) 
		callService(START_STR,Start)
		rospy.sleep(1)
		callService(HOME_STR,HomeArm)
		rospy.sleep(5)
	else:
		print 'Alright! y,z: {},{}'.format(y,z)
		print '{}: {}'.format(frame, trans)

if __name__ == '__main__':
	
	rospy.init_node('monitor_tfs')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	frames = ['/j2n6s300_link_{}'.format(i) for i in range(3,7)] + ['/j2n6s300_end_effector']
	counter = 0	
	while not rospy.is_shutdown():
		print 'Iteration {}===================================='.format(counter)
		counter+=1
		try:
			for frame in frames:
				trans,rot = listener.lookupTransform('/j2n6s300_link_base', frame, rospy.Time(0))
				validate(frame, trans)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print 'tf exception'
		print '===================================='
		rate.sleep()
	pass
