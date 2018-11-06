#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
from kinova_msgs.srv import AddPoseToCartesianTrajectory.srv
import std_msgs.msg
import geometry_msgs.msg


import math
import argparse

def cartesian_trajectories_client(positions, orientations):
    """Send a cartesian trajectory to the action server."""
    server_address = '/' + prefix + '_driver/in/add_pose_to_Cartesian_trajectory'
	rospy.wait_for_service(server_address)
	try:
        addPoseToCartesianTrajectories = rospy.ServiceProxy(server_address, AddPoseToCartesianTrajectories)
    except rospy.ServiceException, e:
        print "Service call failed: %s"

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None
## main
cartesian_trajectories_client(None, None)
