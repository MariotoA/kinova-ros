#! /usr/bin/env python

import rospy
import numpy as np
from pose_action_client import moveArm, currentCartesianCommand,Quaternion2EulerXYZ
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from auto_test import home
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

aux = [0.3, -0.3, 0.5, -3, 0, 0]

if __name__ == '__main__':
	
	rospy.init_node('best_test')
	Y = [-0.3, -0.3, -0.4,-0.4,-0.4,-0.4,-0.4,-0.4,-0.3,-0.3]
	X = [-0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3,0.3,0.4]
	K = {}
	for i in range(len(X)):
		x,y = X[i], Y[i]
		auxi3 = list(aux)
		auxi3[0] = -0.3 if x < -0.1 else 0.3
		moveArm(auxi3, is_relative=False)
		nx, ny = np.random.normal(0, 0.03, 2)
		auxi = list(aux)
		auxi[0:3] = [x+nx,y+ny,0.1]
		auxi2 = list(aux)
		auxi2[0:3] = [x+nx,y+ny,0.3]
		moveArm(auxi2, is_relative=False)
		res,a = moveArm(auxi, is_relative=False)
		s = 'reached' if a == 0 else 'aborted'
		K[(x+nx,y+ny)] = s
		moveArm(auxi2, is_relative=False)
		home()
	print K
	print K.values()
