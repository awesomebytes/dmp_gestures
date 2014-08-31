#!/usr/bin/python
"""
Created on 28/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com


"""
import rospy
from dmp_training import LearnFromEndEffector
from dmp_generation import gestureGeneration

if __name__ == '__main__':
    rospy.init_node("test_train_from_posestampd_")
    rospy.loginfo("Initializing test.")
    motion = "wave"
    lfee = LearnFromEndEffector(['/tf_to_ps'], ['right_arm'])
    lfee.start_learn(motion, motion +"_pose")
    rospy.sleep(5.0)
    motion_dict = lfee.stop_learn()
    rospy.loginfo("Motion dict is: " + str(motion_dict))
    
    gG = gestureGeneration()
    rospy.loginfo("loading gesture from bag end effector")
    gesture_dict = gG.loadGestureFromBagEndEffector(motion_dict['rosbag_name'], ['/tf_to_ps'], ['right_arm'])
    rospy.loginfo("Loaded!")
    rospy.loginfo("gest dict: " + str(gesture_dict))
    
    