#!/usr/bin/env python
import rospy
import numpy as np
from pose_action_client import moveArm, currentCartesianCommand,Quaternion2EulerXYZ
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from math import pi

tolerance_finger_turn = 2
height_off = .45

def fingersAt(current, goal):
    return all([abs(x-y) < tolerance_finger_turn for x,y in zip(current,goal)])

def activeWaitToGripper(fingers_turn):
    while not fingersAt(currentFingerPosition, fingers_turn) :
        rospy.sleep(.2)

def routineA():
    poses = [[.2, -.05, -height_off, -pi/10, 0, pi/10],
                 [.2, 0, 0, 0,0, 0]]
    for p in poses:
        moveArm(p)
    moveFingers([50]*3)
    fingers_turn, _, _ = unitParser('percent', [50]*3, True)
    activeWaitToGripper(fingers_turn)
    print('Closed Fingers')
    poses = [
				[-.2, 0, height_off/2, 0,0, 0],
				[0, .4, 0, 0, 0, 0]]
    for p in poses:
        moveArm(p)
    rospy.sleep(2)
    moveFingers(is_relative=False)
    fingers_turn, _, _ = unitParser('percent', [0]*3, False)
    activeWaitToGripper(fingers_turn)
    poses = [
				[0, -.4, 0, 0, 0, 0],
				[-.2, .05, height_off/2, pi/10, 0, -pi/10]
				]
    
    for p in poses:
        moveArm(p)
def createtraj(pose_goal):
    tot = abs(pose_goal[2] - currentCartesianCommand[2]) / 0.01
    return [np.linspace(x,y,tot).tolist() for x,y in zip(currentCartesianCommand, pose_goal)]

if __name__ == '__main__':
    rospy.init_node('jaco' + 'grab_object_in_fixed_position')
    pose_goal = [0.48021620512, -0.410772562027, 0.0402127914131, 0, 0, 0]
    orientation = [ 0.58049684763, 0.400517255068 ,0.425420224667, 0.567121684551]
    pose_goal[3:] = Quaternion2EulerXYZ(orientation)
    v = createtraj(pose_goal)
    for j in range(len(v[0])):
        p = [0]*6
        for i in range(len(v)):
            p[i] = v[i][j]
        moveArm(p, is_relative=False)
