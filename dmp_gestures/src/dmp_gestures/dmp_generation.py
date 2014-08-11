#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains generation related classes.
"""

import rospy
import subprocess, yaml
import math
import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion
from dmp.srv import GetDMPPlan, GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from dmp.msg import DMPTraj, DMPData, DMPPoint
from sensor_msgs.msg import JointState

DEFAULT_JOINT_STATES = "/joint_states"

class gestureGeneration():
    
    def __init__(self):
        self.head = ['head_1_joint', 'head_2_joint']
        self.torso = ['torso_1_joint', 'torso_2_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint'] # Only the actuated
        self.left_hand_all = self.left_hand + ['hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint',
                          'hand_left_middle_1_joint', 'hand_left_middle_2_joint', 'hand_left_middle_3_joint']
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] # Only the actuated
        self.right_hand_all = self.right_hand + ['hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint',
                               'hand_right_middle_1_joint', 'hand_right_middle_2_joint', 'hand_right_middle_3_joint']
        self.all_joints = self.torso + self.head + self.left_arm + self.right_arm + self.left_hand_all + self.right_hand_all
        self.right_arm_torso = self.torso + self.right_arm
        self.left_arm_torso = self.torso + self.left_arm
        self.both_arms = self.left_arm + self.right_arm
        self.both_arms_torso = self.torso + self.both_arms
        self.gesture_difference = []
        
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

    def printNamesAndValues(self, group='right_arm_torso'):
        """Given a group, print in screen in a pretty way joint names and it's values"""
        names, values = self.getNamesAndMsgList(group=group)
        print "Name =  Joint Value"
        print "================="
        for nam, val in zip(names, values):
            print nam + " = " + str(val)

    def loadGestureFromBagEndEffector(self, bagname):
        """Load gesture from the bag name given """
        # get bag info
        self.info_bag = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagname],
                                                    stdout=subprocess.PIPE).communicate()[0])
        bases_rel_to_time = math.ceil(self.info_bag['duration'] * 20) # empirically for every second 20 bases it's ok
        
        
        # Create a DMP from a 3-D trajectory with orientation
        dims = 6
        dt = 1.0
        K = 100
        D = 2.0 * np.sqrt(K)
        num_bases = bases_rel_to_time
    
        # Fill up traj with real trajectory points  
        traj = []  
        bag = rosbag.Bag(bagname)
        first_point = True
        for topic, msg, t in bag.read_messages(topics=['/teleop_right_hand_pose']):
            p = msg # PoseStamped()
            roll, pitch, yaw = euler_from_quaternion([p.pose.orientation.x,
                                   p.pose.orientation.y,
                                   p.pose.orientation.z,
                                   p.pose.orientation.w])
            traj.append([p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw])
            if first_point:
                # Store first point
                self.gesture_x0 = [p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw]
                first_point = False
        bag.close()
        # Store last point
        self.gesture_goal = [p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw]
        
        # Calculate the difference between initial and final point
        for val1, val2 in zip(self.gesture_x0, self.gesture_goal):                     
            self.gesture_difference.append(val2-val1)
        
        print str(len(traj)) + " points in example traj."
        resp = self.makeLFDRequest(dims, traj, dt, K, D, num_bases)
        #Set it as the active DMP
        self.makeSetActiveRequest(resp.dmp_list)
        self.resp_from_makeLFDRequest = resp
        
    def loadGestureFromBagJointStates(self, bagname, joints):
        """Load gesture from the bag name given """
        # get bag info
        self.info_bag = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagname],
                                                    stdout=subprocess.PIPE).communicate()[0])
        bases_rel_to_time = math.ceil(self.info_bag['duration'] * 20) # empirically for every second 20 bases it's ok
        
        
        # Create a DMP from a X-number of joints trajectory
        dims = len(joints) # number of dims as number of joints
        dt = 1.0
        K = 100
        D = 2.0 * np.sqrt(K)
        num_bases = bases_rel_to_time
    
        # Fill up traj with real trajectory points  
        traj = []  
        bag = rosbag.Bag(bagname)
        first_point = True
        for topic, msg, t in bag.read_messages(topics=[DEFAULT_JOINT_STATES]):
            # Get the joint and it's values...
            js = msg # JointState()
            # Process interesting joints here
            names, positions = self.getNamesAndMsgList(joints, msg)
            # Append interesting joints here
            traj.append(positions)
            if first_point:
                # Store first point
                self.gesture_x0 = positions
                first_point = False
        bag.close()
        # Store last point
        self.gesture_goal = positions
        
        # Calculate the difference between initial and final point
        for val1, val2 in zip(self.gesture_x0, self.gesture_goal):                     
            self.gesture_difference.append(val2-val1)
        
        print str(len(traj)) + " points in example traj. Using " + str(num_bases) + " num_bases"
        resp = self.makeLFDRequest(dims, traj, dt, K, D, num_bases)
        #Set it as the active DMP
        self.makeSetActiveRequest(resp.dmp_list)
        self.resp_from_makeLFDRequest = resp
        
        rospy.loginfo("Response of makeLDFRequest is:\n" + str(self.resp_from_makeLFDRequest) )
        
        rospy.loginfo("Joints:" + str(joints))
        rospy.loginfo("Initial pose:" + str(self.gesture_x0))
        rospy.loginfo("Final pose: " + str(self.gesture_goal))
        rospy.loginfo("DMP result: " + str(self.resp_from_makeLFDRequest))
        

    def makeLFDRequest(self, dims, traj, dt, K_gain,
                       D_gain, num_bases):
        """Learn a DMP from demonstration data """
        demotraj = DMPTraj()
    
        for i in range(len(traj)):
            pt = DMPPoint()
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
    
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
    
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"
    
        return resp

    
    def makeSetActiveRequest(self, dmp_list):
        """Set a DMP as active for planning """
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    
    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter):
        """Generate a plan from a DMP """
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"
    
        return resp


# TODO: Save gesture to yaml with metadata

# TODO: Load gesture from yaml


if __name__ == '__main__':
    rospy.init_node("test_generation_classes")
    rospy.loginfo("Initializing dmp_generation test.")
    gg = gestureGeneration()
    gg.loadGestureFromBagJointStates("uninitialized_rosbag_name.bag", ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint'])
