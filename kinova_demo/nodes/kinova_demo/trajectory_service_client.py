#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
from kinova_msgs.srv import AddPoseToCartesianTrajectory
from math import pi
import std_msgs.msg
import geometry_msgs.msg
from fingers_action_client import gripperGoTo
import math
import argparse
import rosservice

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq
p_go_home = [0, 0, 0, 0,0, 0]
p_increment = [.2, -.1, -.3, 0, 0, 0]
p_increment2 = [x + y for x, y in zip(p_increment, [0, .7, 0, 0, 0, 0])]
p_increment3 = [x + y for x, y in zip(p_increment2, [-.8, 0, 0, 0, 0, 0])]
p_near_floor = [0, -.1, -.2, 0, 0, 0]
p_near_floor2 = [x + y for x, y in zip(p_near_floor, [.1, 0, 0, 0, 0, 0])]
p_rot = [0,0,0,0,2*pi/3,0]


# yaw 2d z roll x pitch y

def cartesian_trajectories_client(positions, orientations):
    """Send a cartesian trajectory to the action server."""
    server_address = '/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory'#'/' + prefix + '_driver/in/add_pose_to_Cartesian_trajectory'
    print('cartesian_trajectories_client function entered')
    print(rosservice.get_service_list())
    rospy.wait_for_service(server_address)
    print('Wait ended')
    try:
        addPoseToCartesianTrajectory = rospy.ServiceProxy(server_address, AddPoseToCartesianTrajectory)
        h = list(currentCartesianCommand)
        print("Service result correctly")
        p = [x + y for x, y in zip(p_near_floor, h)]
        addPoseToCartesianTrajectory(p[0], p[1], p[2], p[3], p[4], p[5]) 
        print "1st pose added to buffer"
        p = [x + y for x, y in zip(p_near_floor2, h)]       
        addPoseToCartesianTrajectory(p[0], p[1], p[2], p[3], p[4], p[5])
        print "2nd pose added to buffer"
        #p = [x + y for x, y in zip(p_increment3, h)]       
        #addPoseToCartesianTrajectory(p[0], p[1], p[2], p[3], p[4], p[5])
        #print "3rd pose added to buffer"
        return 0
        #p = [-x + y for x, y in zip(p_increment, currentCartesianCommand)]
        #addPoseToCartesianTrajectory(p[0], p[1], p[2], p[3], p[4], p[5])
    except rospy.ServiceException as e:
        help(e)
        print "Service call failed: %s"%e
        return 1

def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
    parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
 
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    # print('pose_mq in argumentParser 1: {}'.format(args_.pose_value))  # debug
    return args_

def getcurrentCartesianCommand(prefix_):
    # wait to get current position
    topic_address = '/j2n6s300_driver/out/cartesian_command'#'/' + prefix_ + 'driver/out/cartesian_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
    print("getcurrentCartesianCommand: Subscriber done")
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    print 'position listener obtained message for Cartesian pose. '


def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger
## main

args = argumentParser(None)
kinova_robotTypeParser(args.kinova_robotType)
rospy.init_node(prefix + 'pose_action_client')
getcurrentCartesianCommand(prefix)
cartesian_trajectories_client(None, None)
gripperGoTo([50]*3,args.kinova_robotType)
