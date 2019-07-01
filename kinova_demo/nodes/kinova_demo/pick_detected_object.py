#! /usr/bin/env python

import rospy
import tf
import numpy as np
from pose_action_client import moveArm, currentCartesianCommand,Quaternion2EulerXYZ
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from auto_test import home
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

aux_pose_oriented = [0, 0, 0, -3, 0, 0]
tolerance_finger_turn = 10

def fingersAt(current, goal):
	return all([abs(x-y) < tolerance_finger_turn for x,y in zip(current,goal)])
def activeWaitToGripper(fingers_turn):
	while not fingersAt(currentFingerPosition, fingers_turn) :
		rospy.sleep(.2)

def pick_object(frame, position):
    x,y,z = position
    pose = list(aux_pose_oriented)
    pose[0:3] = [x,y,0.45]
    moveArm(pose,is_relative=False)
    pose[2] = 0.22
    moveArm(pose,is_relative=False)
    pose[2] = z
    res, status = moveArm(pose,is_relative=False)
    if status == 0:
        percent_fing = [45]*3
        moveFingers(percent_fing, is_relative=False)
        fingers_turn, _, _ = unitParser('percent', percent_fing , False)
        activeWaitToGripper(fingers_turn)
        pose[2] = 0.22
        moveArm(pose,is_relative=False)
        pose[2] = 0.45
        moveArm(pose,is_relative=False)
        pose[0] = -pose[0] #+ 0.1
        pose[2] = 0.22
        moveArm(pose,is_relative=False)
        pose[2] = z
        moveArm(pose,is_relative=False)
        percent_fing = [0]*3
        moveFingers(percent_fing, is_relative=False)
        fingers_turn, _, _ = unitParser('percent', percent_fing , False)
        activeWaitToGripper(fingers_turn)
        pose[2] = 0.22
        moveArm(pose,is_relative=False)
    home()

if __name__ == '__main__':	
    rospy.init_node('pick_detected_object')
    frame = 'object_head'
    #frame = 'object_cent'
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    received = False
    while not rospy.is_shutdown() and not received:
        try:
            trans,rot = listener.lookupTransform('root', frame, rospy.Time(0))
            pick_object(frame, trans)
            received = True
        except tf.LookupException as error:
            print error
        except tf.ConnectivityException as error:
            print error
        except tf.ExtrapolationException as error:
            print error
        rate.sleep()
