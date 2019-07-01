#!/usr/bin/env python
import rospy
import numpy as np
from pose_action_client import moveArm, currentCartesianCommand,Quaternion2EulerXYZ
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from math import pi
import tf


aux = [0.3, -0.3, 0.5, -3, 0, 0]
def getPoseObjectSeen(trans):
	refe = list(aux)
	x,y,z = trans[0:]
	refe[0:2] = [x,y]
	refe[2] = 0.6
	moveArm(refe, is_relative=False)
	refe[2] = 0.4
	moveArm(refe, is_relative=False)
	refe[2] = z + 0.2	
	moveArm(refe, is_relative=False)
	
if __name__ == '__main__':
	rospy.init_node('grab_object_in_tf')
	moveArm(aux, is_relative=False)
	#number_object = rospy.get_param("index_object")
	listener = tf.TransformListener()
	try:
		#'cosa_{}'.format(number_object)
		listener.waitForTransform('/cosa_22','/j2n6s300_link_base',rospy.Time(), rospy.Duration(4.0))
		trans,rot = listener.lookupTransform('cosa_22', 'j2n6s300_link_base' , rospy.Time(0))		
		getPoseObjectSeen(trans)
	except (tf.LookupException) as e:
		print("tf.LookupException: {}".format(e))
	except tf.ConnectivityException as e:
		print("OS error: {}".format(e))
	except tf.ExtrapolationException as e:
		print("OS error: {}".format(e))
