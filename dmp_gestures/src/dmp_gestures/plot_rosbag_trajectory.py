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
from scipy.signal import lfilter, firwin
from scipy import signal
from scipy.interpolate import interp1d
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
        fig, plots = plt.subplots(nrows=len(joint_names), ncols=1) # Position plot
        fig_vel, plots_vel = plt.subplots(nrows=len(joint_names), ncols=1) # Velocity plot
        # Set labels
        for plot, plot_vel, joint_name in zip(plots, plots_vel, joint_names):
            plot.set_xlabel('time')
            plot.set_ylabel('position')
            plot.set_title(joint_name)
            plot_vel.set_xlabel('time')
            plot_vel.set_ylabel('velocity')
            plot_vel.set_title(joint_name)
            
        # Read the bag and store the arrays for each plot
        trajectories = []
        velocities = []
        # prepare somewhere to accumulate the data
        for joint in joint_names:
            trajectories.append([])
            velocities.append([])

        num_msgs = 0
        for topic, jsmsg, time in bag.read_messages(topics=['/joint_states']):
            num_msgs += 1
            # jsmsg is a JointState message
            joint_names, joint_vals = self.getJointNamesAndValuesFromJointState(joint_names, jsmsg)
            joint_names, vel_vals = self.getJointNamesAndVelocityValuesFromJointState(joint_names, jsmsg)

            counter = 0
            for joint_val, vel_val in  zip(joint_vals, vel_vals):
                trajectories[counter].append(joint_val)
                velocities[counter].append(vel_val)
                counter += 1
        ticks = range(0, num_msgs)
        for plot, vel_plot, trajectory, velocity in zip(plots, plots_vel, trajectories, velocities):
            plot.plot(ticks, trajectory, 'b-')
            vel_plot.plot(ticks, velocity, 'b-')
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

    def getJointNameAndVelocityValueFromJointState(self, joint_name, js_msg):
        """Given a joint name and a js_msg return it's name and velocity value"""
        #js_msg = JointState()
        joint_val = None
        for name, value in zip(js_msg.name, js_msg.velocity):
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

    def getJointNamesAndVelocityValuesFromJointState(self, joint_names, js_msg):
        """Given a joint names list and a js_msg return all the names with their velocity values"""
        vel_values = []
        for joint_name in joint_names:
            j_name, j_value = self.getJointNameAndVelocityValueFromJointState(joint_name, js_msg)
            vel_values.append(j_value)
        return joint_names, vel_values

    def readAndFilterBag(self, bagname, joint_names=[], cutoff_hz=10.0):
        """Read the bag, do it's normal plot and do a FIR filter for each subplot"""
        bag = rosbag.Bag(bagname)
        sample_rate = 50. # We record at 50Hz the bag from joint_states (it's published at 50Hz)
                #------------------------------------------------
        # Create a FIR filter and apply it to signal.
        #------------------------------------------------
        # The Nyquist rate of the signal.
        nyq_rate = sample_rate / 2.
        # The cutoff frequency of the filter: 6KHz
        cutoff_hz = cutoff_hz
        # Length of the filter (number of coefficients, i.e. the filter order + 1)
        numtaps = 10
        # Use firwin to create a lowpass FIR filter
        fir_coeff = firwin(numtaps, cutoff_hz/nyq_rate)
         

        # Create plots
        fig, plots = plt.subplots(nrows=len(joint_names), ncols=1)
#         print "plots at creation looks like: "
#         print plots
        # Create filtered plots
        fig_filt, plots_filt = plt.subplots(nrows=len(joint_names), ncols=1)
#         print "plots_filt at creation looks like: "
#         print plots_filt
        # Set labels
        for plot, plot_filt, joint_name in zip(plots, plots_filt, joint_names):
            plot.set_xlabel('time')
            plot.set_ylabel('position')
            plot.set_title("orig_" + joint_name)
            plot_filt.set_xlabel('time')
            plot_filt.set_ylabel('filt_position')
            plot_filt.set_title("filt_" + joint_name)
            
#         print "plots after set labels looks like: "
#         print plots
#         print "plots_filt after set labels looks like: "
#         print plots_filt
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
#             print time
#             print joint_names
#             print joint_vals
            counter = 0
            for joint_val in  joint_vals:
                trajectories[counter].append(joint_val)
                counter += 1
        ticks = np.arange(0, num_msgs)
        for plot, trajectory in zip(plots, trajectories):
            plot.plot(ticks, trajectory, 'b-')
            
        print "plots after adding ticks and trajectories looks like:"
        print plots
        # Create filtered trajectories
        # Use lfilter to filter the signal with the FIR filter
        # Other example
        # design filter
        cuttoff_freq = cutoff_hz
        norm_pass = cuttoff_freq/(sample_rate/2)
        norm_pass = 0.06 # designed from real life example using crowd_salute
        print "norm_pass: " + str(norm_pass)
        norm_stop = 1.5*norm_pass
        print "norm_stop: " + str(norm_stop)
        (N, Wn) = signal.buttord(wp=norm_pass, ws=norm_stop, gpass=0.8, gstop=25.0, analog=0)
        (b, a) = signal.butter(N, Wn, btype='low', analog=0, output='ba')
        print("b="+str(b)+", a="+str(a))
        warmup = numtaps - 1
        delay = (warmup / 2) / sample_rate
        for filtered_plot, to_filter_trajectory in zip(plots_filt, trajectories):
            #filtered_signal = lfilter(fir_coeff, 1.0, to_filter_trajectory)
            filtered_signal = lfilter(b, a, to_filter_trajectory)
#             print "to_filter_traj has " + str(len(to_filter_trajectory)) + " elems and looks like:"
#             print to_filter_trajectory
#             print "filtered_singal.tolist() has " + str(len(filtered_signal.tolist())) + " elems and looks like:"
#             print filtered_signal.tolist()
            #filtered_plot.plot(ticks[warmup:]-delay, filtered_signal.tolist()[warmup:]  , 'b-')
            filtered_plot.plot(ticks, filtered_signal.tolist()  , 'b-')
            
        print "plots_filt after adding ticks and trajectories looks like:"
        print plots_filt
        bag.close()
        
#                 # Plot the original and filtered signals.
#         #------------------------------------------------
#          
#         # The first N-1 samples are "corrupted" by the initial conditions
#         warmup = numtaps - 1
#         # The phase delay of the filtered signal
#         delay = (warmup / 2) / sample_rate
#          
#         figure(1)
#         # Plot the original signal
#         plot(t, signal)
#          
#         # Plot the filtered signal, shifted to compensate for the phase delay
#         plot(t-delay, filtered_signal, 'r-')
#          
#         # Plot just the "good" part of the filtered signal.  The first N-1
#         # samples are "corrupted" by the initial conditions.
#         plot(t[warmup:]-delay, filtered_signal[warmup:], 'g', linewidth=4)
#          
#         grid(True)
#          
#         show()
        
        return plt#, plots_filt


    def readAndDownSampleBag(self, bagname, joint_names=[], frequency_to_downsample=10.0):
        """Given a bag downsample it's trajectory knowing it's 50hz and we don't want jerkiness
        then apply cubic spline to the points"""
        bag = rosbag.Bag(bagname)
        # Create plots
        fig, plots = plt.subplots(nrows=len(joint_names), ncols=1) # Position plot
        fig_vel, plots_vel = plt.subplots(nrows=len(joint_names), ncols=1) # Velocity plot
        # Set labels
        for plot, plot_vel, joint_name in zip(plots, plots_vel, joint_names):
            plot.set_xlabel('time')
            plot.set_ylabel('position')
            plot.set_title(joint_name)
            plot_vel.set_xlabel('time')
            plot_vel.set_ylabel('velocity')
            plot_vel.set_title(joint_name)
            
        # Read the bag and store the arrays for each plot
        trajectories = []
        velocities = []
        # prepare somewhere to accumulate the data
        for joint in joint_names:
            trajectories.append([])
            velocities.append([])
    
        num_msgs = 0
        num_data_points = 0
        for topic, jsmsg, time in bag.read_messages(topics=['/joint_states']):
            if num_msgs % frequency_to_downsample != 0:
                num_msgs += 1
                continue
            num_msgs += 1
            num_data_points += 1
            # jsmsg is a JointState message
            joint_names, joint_vals = self.getJointNamesAndValuesFromJointState(joint_names, jsmsg)
            joint_names, vel_vals = self.getJointNamesAndVelocityValuesFromJointState(joint_names, jsmsg)
    
            counter = 0
            for joint_val, vel_val in  zip(joint_vals, vel_vals):
                trajectories[counter].append(joint_val)
                velocities[counter].append(vel_val)
                counter += 1
        ticks = range(0, num_data_points)
        
        # Do cubic spline
        splined_trajs = []
        for traj, vel in zip(trajectories, velocities):
            print "interpolating cubicly..."
            #f = interp1d(ticks, traj, bounds_error=False, kind='cubic')
            f = interp1d(ticks, traj, kind='cubic')
            #newx = np.arange(0, num_msgs)
            ticks_as_array = np.array(ticks)
            newx = np.linspace(ticks_as_array.min(), ticks_as_array.max(), num_msgs)
            print len(newx)
            print newx
            newy = f(newx)
            #print newy
            traj = newy
            splined_trajs.append(newy)

        
        for plot, vel_plot, trajectory, velocity in zip(plots, plots_vel, splined_trajs, velocities):
            ticks_as_array = np.array(ticks)
            newx = np.linspace(ticks_as_array.min(), ticks_as_array.max(), num_msgs)
            print "len newx: " + str(len(newx))
            print "len trajectory: " + str(len(trajectory))
            plot.plot(newx, trajectory, 'b-')
            vel_plot.plot(ticks, velocity, 'b-')
        bag.close()
        return plt
    
    
if __name__=='__main__':
    rtp = rosbagTrajectoryPlotter()
    joints = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                            'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                            'arm_right_7_joint']
    bag_name = 'crowd_salute.bag'
#     plot = rtp.readBagAndCreatePlot(bag_name, joints)
#     plot.show()
    #plot.savefig('plot_' + bag_name + '.png')
    
#     plot = rtp.readAndFilterBag(bag_name, joints, 20.0)
#     print "plot looks like:"
#     print plot
#     plot.show()
    
    plot = rtp.readAndDownSampleBag(bag_name, joints, 15)
    plot.show()
    
    
#     for cutoff in range(1, 51, 10):
#         print float(cutoff)
#         plot = rtp.readAndFilterBag(bag_name, joints, float(cutoff))
#         print "plot looks like:"
#         print plot
#     #     print "filt_plot looks like:"
#     #     print filt_plot
#         plot.show()
#     filt_plot.show()
    
