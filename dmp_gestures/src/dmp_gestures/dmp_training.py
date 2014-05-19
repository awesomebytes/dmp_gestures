#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains training related classes.
"""

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

from helper_functions import moveit_error_dict

DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_JOINT_STATES = "/joint_states"


class LearnFromEndEffector():
    """Manage the learning from 6D pose topics of end effectors"""
    def __init__(self, pose_topics=[], groups=[]):
        """Initialize class.
        @arg pose_topics list of strings containing the topics where
            to get from PoseStamped messages of the end effector pose.
        @arg groups list of strings containing the name of the MoveIt!
            groups to use to get the IK position of the poses given.
        Every pose_topics element will be mapped to a group, i.e.,
        the first topic in pose_topics will be the position to achieve
        by the first group in groups."""
        rospy.loginfo("Init LearnFromEndEffector()")

        # Initialize class vars
        self.start = False
        #TODO: make ik_service_name a param to load from a yaml
        self.ik_service_name = DEFAULT_IK_SERVICE
        self.pose_subs = []

        # Subscribe to PoseStamped topics
        rospy.loginfo("Subscribing to topics...")
        for pose_topic in pose_topics:
            rospy.loginfo("Subscribing to '" + pose_topic + "'...")
            subs = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, callback_args=pose_topic)
            self.pose_subs.append(subs)
            rospy.loginfo("Successful subscription to '" + pose_topic + "'.")

        # Get a ServiceProxy for the IK service
        rospy.loginfo("Waiting for service '" + self.ik_service_name + "'...")
        rospy.wait_for_service(self.ik_service_name)
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, GetPositionIK)
        rospy.loginfo("Successful connection  to '" + self.ik_service_name + "'.")

    def pose_cb(self, data, cb_args):
        """Callback functions for PoseStamped messages.
        cb_args contains the name of the topic to know which callback is which"""
        rospy.loginfo("Received from '" + cb_args + "':\n  " + str(data))
        if self.start:
            rospy.loginfo("Saving stuff")


class LearnFromJointState():
    """Manage the learning from joint positions"""
    def __init__(self, joint_names=[]):
        """Initialize class.
        @arg joint_names list of strings with the name of the
        joints to subscribe on joint_states."""
        rospy.loginfo("Init LearnFromEndEffector()")
        #TODO: make joint states topic a param to change in yaml file
        self.joint_states_topic = DEFAULT_JOINT_STATES
        #TODO: create a subscriber to joint states


if __name__ == '__main__':
    rospy.init_node("test_training_classes")
    rospy.loginfo("Initializing dmp_training test.")

    TEST_TOPIC_1 = "/hydra_right_paddle_pose"
    TEST_TOPIC_2 = "/hydra_left_paddle_pose"
    TEST_GROUP_1 = "right_arm_torso"
    TEST_GROUP_2 = "left_arm"

    # Test if we succesfully create an instance of the class with one topic
    eef_learn = LearnFromEndEffector([TEST_TOPIC_1], [TEST_GROUP_1])
    rospy.sleep(0.5)  # Give a moment to catch up
    pub1 = rospy.Publisher(TEST_TOPIC_1, PoseStamped)
    pub1.publish(PoseStamped())
    pub1.publish(PoseStamped())

    # Test if we succesfully create an instance of the class with two topics
    pub2 = rospy.Publisher(TEST_TOPIC_2, PoseStamped)
    eef_learn_2_topics = LearnFromEndEffector([TEST_TOPIC_1, TEST_TOPIC_2], [TEST_GROUP_1, TEST_GROUP_2])
    rospy.sleep(0.5)  # Give a moment to catch up
    pub2.publish(PoseStamped())
    pub1.publish(PoseStamped())

    rospy.sleep(1)

    """js_learn = LearnFromJointState(['torso_1_joint', 'torso_2_joint',
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint'])
"""
