#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import kinova_msgs.msg
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """

  if goal == None or actual == None:
	return False

  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual, tolerance)
  elif type(actual) is geometry_msgs.msg.PoseStamped:
    return all_close(goal, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal),actual, tolerance)
  elif type(actual) is geometry_msgs.msg.Pose:
    return all_close(goal,pose_to_list(actual), tolerance)
  
  return True
class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self,tolerance):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    self.tolerance = tolerance
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_my_testament',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:

    group_name = "arm"  # "arm" is the group name for kinova-jaco moveit model
	# I suppose it would be best if it was passed through parameter server

    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    # All these assignments are only to make the code above easier to read.
    self.plane_name = '' # We leave this empty so it will be filled with the different added planes later
    self.robot = robot #
    self.scene = scene #
    print scene
    self.group = group #
    # More variables
    self.currentpose = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # home pose 
    self.currentcartesiancommand = self.currentpose
    self.getcurrentToolPose()
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def add_box(self, name, pose, size, frameidref="world",timeout=4):
    if len(size) != 3 :
      print 'Size is a 3-tuple.'
      return
    if len(pose) != 7:
      print 'Pose should be in quaternions (size=7x1)'
      return
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = frameidref # "panda_link0" #
    plane_pose.pose.orientation.x = pose[3]
    plane_pose.pose.orientation.y = pose[4]
    plane_pose.pose.orientation.z = pose[5]
    plane_pose.pose.orientation.w = pose[6]
    plane_pose.pose.position.x = pose[0]
    plane_pose.pose.position.y = pose[1]
    plane_pose.pose.position.z = pose[2]
    self.plane_name = name # should be changed to a list.
    res = self.scene.add_box(self.plane_name, plane_pose,size=size)
    return self.wait_for_state_update(self.plane_name, known = True, timeout=timeout)
    
  def wait_for_state_update(self, name, known=False, obj_is_attached=False, timeout=4):

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.plane_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (obj_is_attached == is_attached) and (known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL
  def getcurrentToolPose(self):
    # wait to get current position
    topic_address = '/j2n6s300_driver/out/cartesian_command' #tool_pose'
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    #rospy.Subscriber(topic_address, geometry_msgs.msg.PoseStamped, self.setcurrentToolPose)
    #rospy.wait_for_message(topic_address, geometry_msgs.msg.PoseStamped)
    print 'position listener obtained message for Cartesian pose. '
  def setcurrentToolPose(self,pose_st):
    #pose = [pose_st.pose.position.x,pose_st.pose.position.y,pose_st.pose.position.z,
    #pose_st.pose.orientation.x,pose_st.pose.orientation.y,pose_st.pose.orientation.z,pose_st.pose.orientation.w]
    #self.currentpose[:3] = pose[:3]
    #self.currentpose[3:] = pose[3:] #Quaternion2EulerXYZ(pose[3:])
    self.currentpose = pose_st
  def setcurrentCartesianCommand(self, feedback):

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        self.currentcartesiancommand[index] = float(temp_str[1])
  def copycurrent(self):
    res = Pose()
    res.position.x, res.position.y, res.position.z = self.currentpose.pose.position.x, self.currentpose.pose.position.y, self.currentpose.pose.position.z
    res.orientation.x,res.orientation.y,res.orientation.z,res.orientation.w = self.currentpose.pose.orientation.x,self.currentpose.pose.orientation.y,self.currentpose.pose.orientation.z,self.currentpose.pose.orientation.w
    return res
  def go_to_pose_goal(self, pose):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    #group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    #plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    #group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    print "Has end effector?",self.group.has_end_effector_link()
    print "Planning frame",self.group.get_planning_frame()
    current_pose = list(self.currentcartesiancommand)#self.copycurrent()
    rospy.sleep(1)
    previous_pose = None
    time_stall = -1
    abort = False
    while not all_close(pose_goal, current_pose, self.tolerance) :#and not abort:
      self.group.set_pose_target(pose_goal)
      plan = self.group.go(wait=True)
      self.group.stop()
      self.group.clear_pose_targets()
      stuck = all_close(current_pose, previous_pose, self.tolerance)
      if stuck and time_stall==-1:
        time_stall = rospy.get_rostime()
      elif stuck:
        abort = (rospy.get_rostime() - time_stall).to_sec() > 1
      else:
        time_stall == -1
        previous_pose = current_pose
      
      current_pose = list(self.currentcartesiancommand)#self.copycurrent()
      rospy.sleep(1)
    print "goal: {}".format(pose_to_list(pose_goal))
    print "curr: {}".format(pose_to_list(current_pose))
    return all_close(pose_goal, current_pose, self.tolerance)

def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_


if __name__ == '__main__':
  try:
    query = 'Please input pose coordinates'
    print query
    b = False
    x,y,z,thx,thy,thz = [input(),input(),input(),input(),input(),input()] if b else [.2,.3,.5,0,0,0]
    tut = MoveGroupPythonIntefaceTutorial(0.05)
    print 'Table is going to be added to the model.'
    size_table = (0.75, 1.40, 0.08)
    pose_table = [0,0,-0.1, 0, 0, 0, 1]
    tut.add_box(name='table', pose= pose_table, size = size_table)
    raw_input("Table added. Please check in rviz and press enter.")
    pose_res = [-.2, 0, .5, 0, 0, 0, 1]
    tut.add_box(name='restriction', pose= pose_res, size = (.1, 1, 1))
    raw_input("Table added. Please check in rviz and press enter.")
    position = [x,y,z]
    eulerori = [thx,thy,thz]
    main_pose =  [0]*7
    for i in range(len(main_pose)):
      main_pose[i] = 0
    main_pose = [0.21335862577 + .2 ,-0.273997783661, 0.489361822605, 0.60780954361, 0.345389455557, 0.366351991892, 0.614051997662]
#position + EulerXYZ2Quaternion(eulerori)
    print tut.go_to_pose_goal(main_pose)
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    pass
