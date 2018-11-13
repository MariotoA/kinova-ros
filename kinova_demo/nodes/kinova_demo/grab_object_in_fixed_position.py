#!/usr/bin/env python
import rospy
from pose_action_client import moveArm
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from math import pi

tolerance_finger_turn = 2
height_off = .45

def fingersAt(current, goal):
    return all([abs(x-y) < tolerance_finger_turn for x,y in zip(current,goal)])

def activeWaitToGripper(fingers_turn):
    while not fingersAt(currentFingerPosition, fingers_turn) :
        rospy.sleep(.2)
if __name__ == '__main__':
    rospy.init_node('jaco' + 'grab_object_in_fixed_position')

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
