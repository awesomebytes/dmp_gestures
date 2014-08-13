#!/usr/bin/python
"""
Created on 13/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com



"""

import rospy
import rosbag
import matplotlib.pyplot as plt
import sys
from sensor_msgs.msg import JointState
from dmp.srv import GetDMPPlan, GetDMPPlanRequest, GetDMPPlanResponse, LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
import numpy as np

class rosbagTrajectoryPlotter():
    def __init__(self):
        print "init rosbagTrajectoryPlotter"
        
    def readBagAndCreatePlot(self, bagname, joint_names=[]):
        """Given a bagname and a joint list return a plot of it"""
        bag = rosbag.Bag(bagname)
        # Create plots
        fig, plots = plt.subplots(nrows=len(joint_names), ncols=1)
        # Set labels
        for plot, joint_name in zip(plots, joint_names):
            plot.set_xlabel('time')
            plot.set_ylabel('position')
            plot.set_title(joint_name)
        # Read the bag and store the arrays for each plot
        trajectories = []
        # prepare somewhere to accumulate the data
        for joint in joint_names:
            trajectories.append([])
#         times = []
        num_msgs = 0
        for topic, jsmsg, time in bag.read_messages(topics=['/joint_states']):
            num_msgs += 1
            # jsmsg is a JointState message
            joint_names, joint_vals = self.getJointNamesAndValuesFromJointState(joint_names, jsmsg)
#             times.append(time)
            print time
            print joint_names
            print joint_vals
            counter = 0
            for joint_val in  joint_vals:
                trajectories[counter].append(joint_val)
                counter += 1
        ticks = range(0, num_msgs)
        for plot, trajectory in zip(plots, trajectories):
            plot.plot(ticks, trajectory, 'b-')
        bag.close()
        return plt
        #plt.show()

    def getJointNameAndValueFromJointState(self, joint_name, js_msg):
        """Given a joint name and a js_msg return it's name and value"""
        #js_msg = JointState()
        joint_val = None
        for name, value in zip(js_msg.name, js_msg.position):
            if name == joint_name:
                joint_val = value
        return joint_name, joint_val

    def getJointNamesAndValuesFromJointState(self, joint_names, js_msg):
        """Given a joint names list and a js_msg return all the names with their values"""
        joint_values = []
        for joint_name in joint_names:
            j_name, j_value = self.getJointNameAndValueFromJointState(joint_name, js_msg)
            joint_values.append(j_value)
        return joint_names, joint_values



if __name__=='__main__':
    rtp = rosbagTrajectoryPlotter()
    joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                            'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                            'arm_right_7_joint']
    bag_name = 'wave.bag'
    plot = rtp.readBagAndCreatePlot(bag_name, joints)
    #plot.show()
    plot.savefig('plot_' + bag_name + '.png')