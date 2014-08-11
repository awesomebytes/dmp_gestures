#!/usr/bin/python
"""
Created on 19/05/14

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

DEFAULT_IK_SERVICE = "/compute_ik"
DEFAULT_JOINT_STATES = "/joint_states"
PLAY_MOTION_AS = "/play_motion"


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

    def start_learn(self):
        self.start = True

    def stop_learn(self):
        self.start = False


class LearnFromJointState():
    """Manage the learning from joint positions"""
    def __init__(self, joint_names=[]):
        """Initialize class.
        @arg joint_names list of strings with the name of the
        joints to subscribe on joint_states."""
        rospy.loginfo("Init LearnFromJointState()")
        #TODO: make joint states topic a param to change in yaml file
        self.joint_states_topic = DEFAULT_JOINT_STATES
        # Creating a subscriber to joint states
        self.start = False
        self.joint_states_subs = rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_cb)
        
        
    def joint_states_cb(self, data):
        """joint_states topic cb"""
        rospy.logdebug("Received joint_states:\n " + str(data))
        if self.start:
            rospy.loginfo("Saving stuff")

    def start_learn(self):
        self.start = True

    def stop_learn(self):
        self.start = False

class RecordFromPlayMotion():
    """Manage the learning from a play motion gesture"""
    def __init__(self):
        rospy.loginfo("Initializing RecordFromPlayMotion")
        rospy.loginfo("Connecting to AS: '" + PLAY_MOTION_AS + "'")
        self.play_motion_as = SimpleActionClient(PLAY_MOTION_AS, PlayMotionAction)
        self.play_motion_as.wait_for_server()
        rospy.loginfo("Connected.")
        import rosbag
        self.current_rosbag_name = "uninitialized_rosbag_name"
        #self.current_rosbag = rosbag.Bag(self.current_rosbag_name + '.bag', 'w')
        self.last_joint_states_data = None
        
    def joint_states_cb(self, data):
        """Callback for joint states topic"""
        if self.start_recording:
            #self.last_joint_states_data = data
            self.current_rosbag.write(DEFAULT_JOINT_STATES, data)
        
    def play_and_record(self, motion_name, joints=[], bag_name="no_bag_name_set"):
        """Play the specified motion and start recording joint states.
        Try to get the joints to record from the metadata of the play_motion gesture
        or, optionally, specify the joints to track"""
        # Check if motion exists in param server
        
        # Get it's info
        
        # check if joints was specified, if not, get the joints to actually save
        
        # Prepare subscriber
        self.joint_states_topic = DEFAULT_JOINT_STATES
        # Creating a subscriber to joint states
        self.start_recording = False
        self.joint_states_subs = rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_cb)
        # play motion
        pm_goal = PlayMotionGoal(motion_name, False, 0)
        self.play_motion_as.send_goal(pm_goal)
        # record bag
        self.current_rosbag_name = bag_name
        self.current_rosbag = rosbag.Bag(self.current_rosbag_name + '.bag', 'w')
        self.start_recording = True
        done_with_motion = False
        while not done_with_motion: 
            state = self.play_motion_as.get_state()
            #rospy.loginfo("State is: " + str(state) + " which is: " + goal_status_dict[state])
            if state == GoalStatus.ABORTED or state == GoalStatus.SUCCEEDED:
                done_with_motion = True
                self.start_recording = False
        # when motion finishes close bag
        self.current_rosbag.close()

if __name__ == '__main__':
    rospy.init_node("test_training_classes")
    rospy.loginfo("Initializing dmp_training test.")

    TEST_TOPIC_1 = "/hydra_right_paddle_pose"
    TEST_TOPIC_2 = "/hydra_left_paddle_pose"
    TEST_GROUP_1 = "right_arm_torso"
    TEST_GROUP_2 = "left_arm"

    # Test if we successfully create an instance of the class with one topic
    eef_learn = LearnFromEndEffector([TEST_TOPIC_1], [TEST_GROUP_1])
    rospy.sleep(0.5)  # Give a moment to catch up
    pub1 = rospy.Publisher(TEST_TOPIC_1, PoseStamped)
    pub1.publish(PoseStamped())
    pub1.publish(PoseStamped())

    # Test if we successfully create an instance of the class with two topics
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
