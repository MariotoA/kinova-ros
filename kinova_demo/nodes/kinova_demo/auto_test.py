#! /usr/bin/env python

import rospy
import numpy as np
from pose_action_client import moveArm, currentCartesianCommand,Quaternion2EulerXYZ
from fingers_action_client import moveFingers, currentFingerPosition,unitParser
from kinova_msgs.srv import HomeArm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

aux = [0, 0, 0, -3, 0, 0]#[0, 0, 0, 3, 0, 1.8]
tolerance_finger_turn = 10
height_off = .45
poses = 0
ploth, plotl = {},{}
poseh, posel =  {}, {}
def fingersAt(current, goal):
	return all([abs(x-y) < tolerance_finger_turn for x,y in zip(current,goal)])
def activeWaitToGripper(fingers_turn):
	while not fingersAt(currentFingerPosition, fingers_turn) :
		rospy.sleep(.2)

def next(X,Y, i):
	return [X[i+1], Y[i+1]] if i+1 < X.size else [X[0], Y[0]] 
def next_circ(x,y, st):
	r = np.sqrt(x**2 + y**2)
	th0 = np.atan2(y,x)
	th1 = th0 + st
	return [r*np.cos(th1),r*np.sin(th1)] if th0 < np.pi else [(r+.1)*np.cos(th0),(r+.1)*np.sin(th0)]
def home():
	address = "/j2n6s300_driver/in/home_arm"
	rospy.wait_for_service(address)
	try:
		homeArm = rospy.ServiceProxy(address, HomeArm)
		return homeArm()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def add(dicti, x, y,i):
	if not (x,y) in dicti:
		dicti[(x,y)] = [0]*3
	dicti[(x,y)][i] += 1
def savepose(dicti, X,Y, pose):
	x,y,z = [pose.pose.pose.position.x,
	pose.pose.pose.position.y,pose.pose.pose.position.z]
	thx,thy,thz = Quaternion2EulerXYZ([pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,
	pose.pose.pose.orientation.z,pose.pose.pose.orientation.w])
	posenum = [x,y,z,thx, thy, thz,1]
	if not (X,Y) in dicti:
		dicti[(X,Y)] = []
	dicti[(X,Y)].append(posenum)

def pick_place(X0,X1):
	global aux
	global poses
	global plotl
	global ploth
	p = [X0, X1]
	for i in range(2):
		x,y = p[i]
		pose = list(aux)
		pose[:3] = [x,y,0.5]
		result,reached = moveArm(pose, is_relative=False)
		add(ploth, x, y, reached)
		savepose(poseh, x,y, result)
		pose1 = pose[:]
		pose1[2] = 0
		result,reached = moveArm(pose1, is_relative=False)
		add(plotl, x, y, reached)
		savepose(posel, x,y, result)
		percent_fing = [(1-i)*50]*3
		moveFingers(percent_fing, is_relative=False)
		fingers_turn, _, _ = unitParser('percent', percent_fing , False)
		activeWaitToGripper(fingers_turn)
		result,reached = moveArm(pose, is_relative=False)
		add(ploth, x, y, reached)
		savepose(poseh, x,y, result)
		home()
def zeta(collect, x,y):
	return collect[(x,y)] if (x,y) in collect else 0

def test_grid_against_table():
	xlimm,xlimM = -.4, .5 # -.4, -.3#
	ylimm, ylimM = -.7, -.3 #-.5, -.4#
	X,stepX = np.linspace(xlimm, xlimM, np.round(float(abs(xlimm-xlimM)*10)), endpoint = False, retstep=True)
	Y,stepY = np.linspace(ylimM, ylimm, np.round(float(abs(ylimm-ylimM)*10)), endpoint= False, retstep=True)
	X,Y = np.meshgrid(X,Y)
	xpos, ypos =  X.ravel(), Y.ravel()
	[pick_place([xpos[i],ypos[i]], next(xpos,ypos,i)) for i in range(xpos.size)]
	L = []
	for i in range(xpos.size):
		x,y = xpos[i], ypos[i]
		hsl,hsh,ph,pl = plotl[(x,y)], ploth[(x,y)], np.median(np.asarray(poseh[(x,y)]), axis=0), np.median(np.asarray(posel[(x,y)]),axis=0)
		L += [x,y, hsl[0], hsl[1], hsl[2],hsh[0],hsh[1],hsh[2],
						ph[0],ph[1],ph[2],ph[3],ph[4],ph[5],ph[6],
						 pl[0],pl[1],pl[2],pl[3],pl[4],pl[5],pl[6]]
	save = np.asarray(L)
	np.savetxt("res.csv", save, delimiter=",")

def translate(x):
	return []
def test_circ():
	r,th,stepTh = .4, np.linspace(0, np.pi, 10, endpoint = True, retstep=True)
	X,Y = r*np.cos(th), r*np.sin(th)
	[pick_place([x,y], next_circ(x,y,stepTh)) for x,y in zip(X,Y)]
	plt.subplot(111)
	Z = [translate(z) for (x,y), z in plot.items()]
	plt.scatter(X,Y)
	

if __name__ == '__main__':
	rospy.init_node('test_auto')
	test_grid_against_table()
