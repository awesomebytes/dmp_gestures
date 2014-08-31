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
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult
from helper_functions import moveit_error_dict, goal_status_dict
import matplotlib.pyplot as plt
import time
import tf

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
#         #TODO: make ik_service_name a param to load from a yaml
#         self.ik_service_name = DEFAULT_IK_SERVICE
        self.pose_subs = []
        self.pose_accumulators = []
        self.pose_topics = pose_topics
        self.groups = groups
        self.motion_name = "no_motion_name"
        self.current_bag_name = "no_bag_name"
        # Subscribe to PoseStamped topics
        rospy.loginfo("Subscribing to topics...")
        for idx, pose_topic in enumerate(pose_topics):
            rospy.loginfo("Subscribing to '" + pose_topic + "'...")
            subs = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, callback_args=idx)
            self.pose_subs.append(subs)
            rospy.loginfo("Successful subscription to [" + str(idx) + "] '" + pose_topic + "'.")
            self.pose_accumulators.append([])

#         # Get a ServiceProxy for the IK service
#         rospy.loginfo("Waiting for service '" + self.ik_service_name + "'...")
#         rospy.wait_for_service(self.ik_service_name)
#         self.ik_serv = rospy.ServiceProxy(self.ik_service_name, GetPositionIK)
#         rospy.loginfo("Successful connection  to '" + self.ik_service_name + "'.")

    def pose_cb(self, data, cb_args):
        """Callback functions for PoseStamped messages.
        cb_args contains the idx of the topic to know which callback is which"""
        rospy.loginfo("Received from [" + str(cb_args) + "] " + self.pose_topics[cb_args] + ":\n  " + str(data))
        if self.start:
            self.pose_accumulators[cb_args].append(data)

    def start_learn(self, motion_name, bag_name):
        self.motion_name = motion_name
        self.current_bag_name = bag_name
        self.start = True

    def stop_learn(self):
        self.start = False
        for idx, pose_topic in enumerate(self.pose_topics):
            rospy.loginfo("Unsubscribing to '" + pose_topic + "'...")
            self.pose_subs[idx].unregister()
        # Write rosbag
        rospy.loginfo("Recording in bag!")
        self.current_rosbag = rosbag.Bag(self.current_bag_name + '.bag', 'w')
        for idx, poselist in enumerate(self.pose_accumulators):
            for ps in poselist:
                # pose = PoseStamped()
                self.current_rosbag.write(self.pose_topics[idx], ps, t=ps.header.stamp)
        self.current_rosbag.close()
        rospy.loginfo("Motion finished and closed bag.")
        motion_data = {'motion_name' : self.motion_name,
                       'groups' : self.groups, # Get joints from group?
                       'rosbag_name': self.current_bag_name + '.bag'}

        return motion_data


class LearnFromJointState():
    """Manage the learning from joint positions"""
    def __init__(self):
        """Initialize class.
        @arg joint_names list of strings with the name of the
        joints to subscribe on joint_states."""
        rospy.loginfo("Init LearnFromJointState()")
        #TODO: make joint states topic a param to change in yaml file
        self.joint_states_topic = DEFAULT_JOINT_STATES
        # Creating a subscriber to joint states
        self.start_recording = False
        self.joint_states_subs = rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_cb)
        rospy.loginfo("Connected.")
        self.current_rosbag_name = "uninitialized_rosbag_name"
        self.last_joint_states_data = None
        self.joint_states_accumulator = []
        self.motion_name = "no_motion_name"
        self.joints_to_record = []

        
    def joint_states_cb(self, data):
        """joint_states topic cb"""
        rospy.logdebug("Received joint_states:\n " + str(data))
        if self.start_recording:
            self.joint_states_accumulator.append(data)

    def start_learn(self, motion_name, joints=[], bag_name="no_bag_name_set"):
        """Start the learning writting in the accumulator of msgs"""
        self.current_rosbag_name = bag_name
        self.start_recording = True
        if len(joints) > 0:
            self.joints_to_record = joints
        else:
            rospy.logerr("No joints provided to record, aborting")
            return

    def stop_learn(self):
        """Stop the learning writting the bag into disk and returning the info of the motion"""
        self.start_recording = False
        self.joint_states_subs.unregister()
        rospy.loginfo("Recording in bag!")
        self.current_rosbag = rosbag.Bag(self.current_rosbag_name + '.bag', 'w')
        for js_msg in self.joint_states_accumulator:
            self.current_rosbag.write(DEFAULT_JOINT_STATES, js_msg, t=js_msg.header.stamp)
        self.current_rosbag.close()
        rospy.loginfo("Motion finished and closed bag.")
        motion_data = {'motion_name' : self.motion_name,
                       'joints' : self.joints_to_record,
                       'rosbag_name': self.current_rosbag_name + '.bag'}
        return motion_data

class LearnFromRobotTrajectory():
    """Manage the learning from robot trajectory message"""
    def __init__(self):
        """Initialize class.
        @arg joint_names list of strings with the name of the
        joints to subscribe on joint_states."""
        rospy.loginfo("Init LearnFromRobotTrajectory()")
        # Creating a subscriber to joint states
        self.start_recording = False
        self.current_rosbag_name = "uninitialized_rosbag_name"
        self.joint_states_accumulator = []
        self.motion_name = "no_motion_name"
        self.joints_to_record = []

    def start_learn(self, robot_traj, motion_name, joints=[], bag_name="no_bag_name_set"):
        """Start the learning writting in the accumulator of msgs from the RobotTrajectory msg"""
        self.current_rosbag_name = bag_name
        self.start_recording = True
        if len(joints) > 0:
            self.joints_to_record = joints
        else:
            rospy.logerr("No joints provided to record, aborting")
            return
        
        rt =RobotTrajectory()
        for point in robot_traj.joint_trajectory.points:
            js = JointState()
            js.name = robot_traj.joint_trajectory.joint_names
            js.header.stamp = point.time_from_start
            js.position = point.positions
            self.joint_states_accumulator.append(js)
        

    def stop_learn(self):
        """Stop the learning writting the bag into disk and returning the info of the motion"""
        self.start_recording = False
        rospy.loginfo("Recording in bag!")
        self.current_rosbag = rosbag.Bag(self.current_rosbag_name + '.bag', 'w')
        for js_msg in self.joint_states_accumulator:
            self.current_rosbag.write(DEFAULT_JOINT_STATES, js_msg, t=js_msg.header.stamp)
        self.current_rosbag.close()
        rospy.loginfo("Motion finished and closed bag.")
        motion_data = {'motion_name' : self.motion_name,
                       'joints' : self.joints_to_record,
                       'rosbag_name': self.current_rosbag_name + '.bag'}
        return motion_data




class RecordFromPlayMotion():
    """Manage the learning from a play motion gesture"""
    def __init__(self):
        rospy.loginfo("Initializing RecordFromPlayMotion")
        rospy.loginfo("Connecting to AS: '" + PLAY_MOTION_AS + "'")
        self.play_motion_as = SimpleActionClient(PLAY_MOTION_AS, PlayMotionAction)
        self.play_motion_as.wait_for_server()
        rospy.loginfo("Connected.")
        self.current_rosbag_name = "uninitialized_rosbag_name"
        self.last_joint_states_data = None
        self.joint_states_accumulator = []
        #self.time_accumulator = []
        
    def joint_states_cb(self, data):
        """Callback for joint states topic"""
        if self.start_recording:
            #self.last_joint_states_data = data
            #self.current_rosbag.write(DEFAULT_JOINT_STATES, data)
            self.joint_states_accumulator.append(data)
            #self.time_accumulator.append(rospy.Time.now())
        
    def play_and_record(self, motion_name, joints=[], bag_name="no_bag_name_set"):
        """Play the specified motion and start recording joint states.
        Try to get the joints to record from the metadata of the play_motion gesture
        or, optionally, specify the joints to track"""
        # Check if motion exists in param server
        PLAYMOTIONPATH = '/play_motion/motions/'
        if not rospy.has_param(PLAYMOTIONPATH + motion_name):
            rospy.logerr("Motion named: " + motion_name + " does not exist in param server at " + PLAYMOTIONPATH + motion_name)
            return
        else:
            rospy.loginfo("Found motion " + motion_name + " in param server at " + PLAYMOTIONPATH + motion_name)
        # Get it's info
        motion_info = rospy.get_param(PLAYMOTIONPATH + motion_name)
        # check if joints was specified, if not, get the joints to actually save
        if len(joints) > 0:
            joints_to_record = joints
        else:
            joints_to_record = motion_info['joints']
        rospy.loginfo("Got joints: " + str(joints_to_record))
        # Prepare subscriber
        self.joint_states_topic = DEFAULT_JOINT_STATES
        # Creating a subscriber to joint states
        rospy.loginfo("Subscribing to joint states...")
        self.start_recording = False
        self.joint_states_subs = rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_cb)
        # play motion
        rospy.loginfo("Playing motion!")
        pm_goal = PlayMotionGoal(motion_name, False, 0)
        self.play_motion_as.send_goal(pm_goal)
        
        self.start_recording = True
        done_with_motion = False
        while not done_with_motion: 
            state = self.play_motion_as.get_state()
            #rospy.loginfo("State is: " + str(state) + " which is: " + goal_status_dict[state])
            if state == GoalStatus.ABORTED or state == GoalStatus.SUCCEEDED:
                done_with_motion = True
                self.start_recording = False
            elif state != GoalStatus.PENDING and state != GoalStatus.ACTIVE:
                rospy.logerr("We got state " + str(state) + " unexpectedly, motion failed. Aborting.")
                self.joint_states_subs.unregister()
                return None
            rospy.sleep(0.1)
        # when motion finishes close bag
        self.joint_states_subs.unregister()
        # Now write data to rosbag
        rospy.loginfo("Recording in bag!")
        self.current_rosbag_name = bag_name
        self.current_rosbag = rosbag.Bag(self.current_rosbag_name + '.bag', 'w')
        for js_msg in self.joint_states_accumulator:
            self.current_rosbag.write(DEFAULT_JOINT_STATES, js_msg, t=js_msg.header.stamp)
        self.current_rosbag.close()
        rospy.loginfo("Motion finished and closed bag.")
        motion_data = {'motion_name' : motion_name,
                       'joints' : joints_to_record,
                       'rosbag_name': self.current_rosbag_name + '.bag'}
        return motion_data


class RecordPoseStampedFromPlayMotion():
    """Manage the learning from a play motion gesture"""
    def __init__(self):
        rospy.loginfo("Initializing RecordFromPlayMotion")
        rospy.loginfo("Connecting to AS: '" + PLAY_MOTION_AS + "'")
        self.play_motion_as = SimpleActionClient(PLAY_MOTION_AS, PlayMotionAction)
        self.play_motion_as.wait_for_server()
        rospy.loginfo("Connected.")
        self.current_rosbag_name = "uninitialized_rosbag_name"
        self.lfee = LearnFromEndEffector(['/tf_to_ps'], ['right_arm'])
        
    def play_and_record(self, motion_name, groups=['right_arm'], bag_name="no_bag_name_set"):
        """Play the specified motion and start recording poses.
        Try to get the joints to record from the metadata of the play_motion gesture
        or, optionally, specify the joints to track"""
        # Check if motion exists in param server
        PLAYMOTIONPATH = '/play_motion/motions/'
        if not rospy.has_param(PLAYMOTIONPATH + motion_name):
            rospy.logerr("Motion named: " + motion_name + " does not exist in param server at " + PLAYMOTIONPATH + motion_name)
            return
        else:
            rospy.loginfo("Found motion " + motion_name + " in param server at " + PLAYMOTIONPATH + motion_name)
        # Get it's info
        motion_info = rospy.get_param(PLAYMOTIONPATH + motion_name)
        # check if joints was specified, if not, get the joints to actually save
        if len(groups) > 0:
            joints_to_record = groups
        else:
            joints_to_record = motion_info['groups']
        rospy.loginfo("Got groups: " + str(joints_to_record))

        # play motion
        rospy.loginfo("Playing motion!")
        pm_goal = PlayMotionGoal(motion_name, False, 0)
        self.play_motion_as.send_goal(pm_goal)
        self.lfee.start_learn(motion_name, bag_name)
        done_with_motion = False
        while not done_with_motion: 
            state = self.play_motion_as.get_state()
            #rospy.loginfo("State is: " + str(state) + " which is: " + goal_status_dict[state])
            if state == GoalStatus.ABORTED or state == GoalStatus.SUCCEEDED:
                done_with_motion = True
            elif state != GoalStatus.PENDING and state != GoalStatus.ACTIVE:
                rospy.logerr("We got state " + str(state) + " unexpectedly, motion failed. Aborting.")
                return None
            rospy.sleep(0.1)
        # data is written to rosbag in here
        motion_data = self.lfee.stop_learn()

        return motion_data



class recordedTrajectoryPlotter():
    def __init__(self):
        print "recordedTrajectoryPlotter initialized."
        
    def rosbagToPlot(self, rosbag_name, joint_names):
        """Given a rosbag with joint states topic make a plot of the trajectory"""
        fig, plots = plt.subplots(nrows=len(joint_names), ncols=1)
        # Set labels
        for plot, joint_name in zip(plots, joint_names):
            plot.set_xlabel('time')
            plot.set_ylabel('position')
            plot.set_title(joint_name)
            
        # Fill up traj with real trajectory points  
        # Read the bag and store the arrays for each plot
        trajectories = []
        # prepare somewhere to accumulate the data
        for joint in joint_names:
            trajectories.append([])
        bag = rosbag.Bag(rosbag_name)
        num_points = 0
        for topic, msg, t in bag.read_messages(topics=[DEFAULT_JOINT_STATES]):
            num_points += 1
            # Get the joint and it's values...
            js = msg # JointState()
            # Process interesting joints here
            names, positions = self.getNamesAndMsgList(joint_names, msg)
            # Append interesting joints here
            counter = 0
            for joint_val in  positions:
                trajectories[counter].append(joint_val)
                counter += 1
        bag.close()

        ticks = range(0, num_points)
        for plot, trajectory in zip(plots, trajectories):
            plot.plot(ticks, trajectory, 'b-')
        return plt 

    def getNamesAndMsgList(self, joints, joint_state_msg):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """

        list_to_iterate = joints
        curr_j_s = joint_state_msg
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))

        return list_to_iterate, msg_list

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
