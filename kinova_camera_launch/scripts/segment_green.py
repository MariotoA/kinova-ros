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
	
	rospy.init_node('segment_images')
	rospy.Subscriber("chatter", String, callback)
	
