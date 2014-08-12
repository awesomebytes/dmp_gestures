#!/usr/bin/python
"""
Created on 12/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com


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
from dmp_generation import gestureGeneration

DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_JOINT_STATES = "/joint_states"
PLAY_MOTION_AS = "/play_motion"



if __name__ == '__main__':
    rospy.init_node("test_recordfromplaymotion_and_generate")
    rospy.loginfo("Initializing test.")
    motion = "home"
    rfpm = RecordFromPlayMotion()
    rfpm.play_and_record(motion, bag_name = motion)
    rospy.loginfo("Test done, there should be a bag named: " + motion + ".bag")
    
    gG = gestureGeneration()
    if not gG.loadGestureYAML(motion +".yaml"):
        # If the gesture does not exist
        gesture_dict = gG.loadGestureFromBagJointStates(motion +".bag", ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']) # TODO: get from the gesture name the joints
    # Now given a gesture loaded in the DMP server, let's ask for a new plan... from the original points so it should be 
    # almost equal to the trained one
    rospy.loginfo("Got gesture:\n" + str(gesture_dict))
#     Looks like:
#             gesture_dict = {"name": name,
#                         "joints" : joints,
#                         "initial_pose" : initial_pose,
#                         "final_pose" : final_pose,
#                         "computed_dmp" : computed_dmp}
    plan = gG.getPlan(gesture_dict["initial_pose"],gesture_dict["final_pose"], seg_length=int(gesture_dict["duration"]))
    rospy.loginfo("Got plan:\n" + str(plan))
    
    
    