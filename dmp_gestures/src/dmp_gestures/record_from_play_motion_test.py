#!/usr/bin/python
"""
Created on 11/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains training related classes.
"""

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult
from helper_functions import moveit_error_dict, goal_status_dict
from dmp_training import RecordFromPlayMotion

DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_JOINT_STATES = "/joint_states"
PLAY_MOTION_AS = "/play_motion"

if __name__ == '__main__':
    rospy.init_node("test_recordfromplaymotion_class")
    rospy.loginfo("Initializing test.")
    gesture_name = 'wave'
    rfpm = RecordFromPlayMotion()
    motiondata = rfpm.play_and_record(gesture_name, bag_name = gesture_name)
    rospy.loginfo("Test done, there should be a bag named: " + gesture_name + ".bag")
    