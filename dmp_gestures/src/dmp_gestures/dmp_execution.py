#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains execution related classes.
"""

import rospy
from dmp.srv import GetDMPPlan, GetDMPPlanRequest, GetDMPPlanResponse,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory, DisplayRobotState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, ExecuteKnownTrajectoryResponse
from sensor_msgs.msg import JointState
from kinematics_interface import StateValidity
from moveit_msgs.msg._DisplayRobotState import DisplayRobotState
import time
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes
import copy

DEFAULT_JOINT_STATES = '/joint_states'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'

def createExecuteKnownTrajectoryRequest(trajectory, wait_for_execution=True):
    """Create a ExecuteKnownTrajectoryRequest from the given data,
    trajectory must be a RobotTrajectory probably filled from a GetCartesianPath call"""
    ektr = ExecuteKnownTrajectoryRequest()
    ektr.trajectory = trajectory
    ektr.wait_for_execution = wait_for_execution
    return ektr

class gestureExecution():
    
    def __init__(self):
        rospy.loginfo("Initializing gestureExecution")
        rospy.loginfo("Connecting to MoveIt! known trajectory executor server '" + EXECUTE_KNOWN_TRAJ_SRV + "'...")
        self.execute_known_traj_service = rospy.ServiceProxy(EXECUTE_KNOWN_TRAJ_SRV, ExecuteKnownTrajectory)
        self.execute_known_traj_service.wait_for_service()
        rospy.loginfo("Connected.")
        self.sv = StateValidity()
        self.robot_state_collision_pub = rospy.Publisher('/robot_collision_state', DisplayRobotState)
        rospy.sleep(0.1) # Give time to the publisher to register
        
    def robotTrajectoryFromPlan(self, plan, joint_names):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotState to be able to visualize what it consists and also
        to be able to send it to execution"""
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = joint_names
        for point, time in zip(plan.plan.points, plan.plan.times):
            jtp = JointTrajectoryPoint()
            jtp.positions = point.positions
            jtp.velocities = point.velocities
            jtp.time_from_start = rospy.Duration( time )
            #rospy.logwarn("jtp is: " + str(jtp))
            rt.joint_trajectory.points.append(jtp)
        return rt
    
    def checkTrajectoryValidity(self, robot_trajectory, groups=[]):
        """Given a robot trajectory, deduce it's groups and check it's validity on each point of the traj
        returns True if valid, False otherwise
        It's considered not valid if any point is not valid"""
        #r = RobotTrajectory()
        init_time = time.time()
        if len(groups) > 0:
            groups_to_check = groups
        else:
            groups_to_check = ['both_arms_torso'] # Automagic group deduction... giving a group that includes everything
        for traj_point in robot_trajectory.joint_trajectory.points:
            rs = RobotState()
            rs.joint_state.name = robot_trajectory.joint_trajectory.joint_names
            rs.joint_state.position = traj_point.positions
            for group in groups_to_check:
                result = self.sv.getStateValidity(rs, group)#, constraints)
                if not result.valid: # if one point is not valid, the trajectory is not valid
                    rospy.logerr("Trajectory is not valid at point (RobotState):" + str(rs) + "with result of StateValidity: " + str(result))
                    rospy.logerr("published in /robot_collision_state the conflicting state")
                    drs = DisplayRobotState()
                    drs.state = rs
                    self.robot_state_collision_pub.publish(drs)
                    return False
        fin_time = time.time()
        rospy.logwarn("Trajectory validity of " + str(len(robot_trajectory.joint_trajectory.points)) + " points took " + str(fin_time - init_time))
        return True
        
    
    def displayTrajFromPlan(self, plan, joint_names, initial_state):
        """Given a plan (GetDMPPlanResponse), joint_names list and an initial_state as RobotState
        Create a displayTraj message"""
        dt = DisplayTrajectory()
        dt.trajectory.append( self.robotTrajectoryFromPlan(plan, joint_names) )
        dt.trajectory_start = self.robotStateFromJoints(joint_names, initial_state)
        dt.model_id = "reem"
        return dt

    def robotStateFromJoints(self, joint_names, initial_state):
        """Given joint names and configs return a robotstate message"""
        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = initial_state
        return rs

    def sendTrajectory(self, robot_trajectory, wait_for_execution=True):
        """Given a RobotTrajectory send it to the controllers firstly checking it's trajectory validity"""
        if self.checkTrajectoryValidity(robot_trajectory):
            ektr = createExecuteKnownTrajectoryRequest(robot_trajectory, wait_for_execution)
            rospy.loginfo("Sending trajectory...")
            time_init = time.time()
            ros_time_init = rospy.Time.now()
            self.execute_known_traj_service.call(ektr)
            ros_time_fin = rospy.Time.now()
            time_fin = time.time()
            ros_time_diff = ros_time_fin - ros_time_init
            
            rospy.loginfo("Finished trajectory! The service call took (realtime): " + str(time_fin - time_init) + " (ros time): " + str(ros_time_diff.to_sec()) + "s " + str(ros_time_diff.to_nsec()) + "ns")
        else:
            rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
        
    def sliceTrajectoryAndSend(self, robot_trajectory, slice_time=2.0, call_delay=0.3, wait_for_execution=True):
        """DOES NOT WORK AS THE EXECUTION SERVICE STOPS BETWEEN TRAJECTORIESGiven a RobotTrajectory send it to the controllers dividing it in slices of slice_time seconds
        to give an "online" trajectory validity checking
        By empirical results the delay between the trajectory real duration is about 0.3s (that is the time
        needed for the moveit node to deduce which controllers to use and actually send the trajectory)"""
        sliced_trajectories = []
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = robot_trajectory.joint_trajectory.joint_names
        # We know times start from 0.0 (as we generated them previously)
        next_slice_time = rospy.Duration(slice_time) # first slice at slice_time
#         slice_counter = 0
        for idx, point in enumerate(robot_trajectory.joint_trajectory.points):
            rt.joint_trajectory.points.append(point)
#             print "point time: " + str(point.time_from_start.to_sec())
#             print "slice_time: " + str(slice_time)
#             print "next_slice_time: " + str(next_slice_time.to_sec())
            if next_slice_time <= point.time_from_start: # Every time we get into the slicing time we generate a new traj
#                 print "   ----SLICING----"
#                 print "Adding traj with " + str(len(rt.joint_trajectory.points)) + " points"
#                 slice_counter += 1
#                 print "This is slice #" + str(slice_counter)
                sliced_trajectories.append(rt)
                rt = RobotTrajectory() # Create a new one
                rt.joint_trajectory.joint_names = robot_trajectory.joint_trajectory.joint_names
                next_slice_time = point.time_from_start + rospy.Duration(slice_time)
        
        rospy.loginfo("We sliced the trajectory of " + str(len(robot_trajectory.joint_trajectory.points)) + " points in "\
                      + str(len(sliced_trajectories)) + " trajectories of max " + str(len(sliced_trajectories[0].joint_trajectory.points)) + " points" )
        last_sent_time = None
        for id, traj in enumerate(sliced_trajectories):
            if self.checkTrajectoryValidity(traj):
                ektr = createExecuteKnownTrajectoryRequest(traj, False) # This was false
                if id > 0:
                    # Wait for the trajectory to be executed until the point we can send the next one...
                    rospy.loginfo("Waiting for last trajectory to achieve the time where sending the next trajectory will connect them nicely")
                    pre_wait_time = rospy.Time.now()
                    while rospy.Time.now() - last_sent_time < rospy.Duration(slice_time):# - call_delay):
                        rospy.sleep(0.001)
                    after_wait_time = rospy.Time.now()
                    rospy.loginfo("We waited for " + str((after_wait_time - pre_wait_time).to_sec()))
                rospy.loginfo("Sending trajectory... #" + str(id) + "/" + str(len(sliced_trajectories)))
                if last_sent_time != None:
                    rospy.loginfo("We took from last sent time to now: " + str((rospy.Time.now() -last_sent_time).to_sec()))
                last_sent_time = rospy.Time.now()
                trajectory_executed = False
                while not trajectory_executed:
                    answer = self.execute_known_traj_service.call(ektr)
                    #ExecuteKnownTrajectoryResponse.error_code.val
                    if answer.error_code.val == MoveItErrorCodes.SUCCESS:
                        trajectory_executed = True
                    else:
                        print "not sleeping"
                        #rospy.sleep(0.5)
                    rospy.logwarn("Answer of execute service: " + str(answer))
                rospy.sleep(slice_time - call_delay)
                # We actually should rewrite all the times to match with the necessary offset...
                # Let's try this first!

                rospy.loginfo("Sent trajectory!")
            else:
                rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
                break

    def sliceTrajectoryRewritingTimesAndSend(self, robot_trajectory, slice_time=4.0, call_delay=0.008, wait_for_execution=True):
        """DOES NOT WORK AS THE EXECUTION SERVICE STOPS BETWEEN TRAJECTORIES...Given a RobotTrajectory send it to the controllers dividing it in slices of slice_time seconds
        to give an "online" trajectory validity checking
        By empirical results the delay between the trajectory real duration is about 0.3s (that is the time
        needed for the moveit node to deduce which controllers to use and actually send the trajectory)"""
        sliced_trajectories = []
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = robot_trajectory.joint_trajectory.joint_names
        # We know times start from 0.0 (as we generated them previously)
        next_slice_time = rospy.Duration(slice_time) # first slice at slice_time
#         slice_counter = 0
        for idx, point in enumerate(robot_trajectory.joint_trajectory.points):
#             print "point time: " + str(point.time_from_start.to_sec())
#             print "slice_time: " + str(slice_time)
#             print "next_slice_time: " + str(next_slice_time.to_sec())
            if next_slice_time <= point.time_from_start: # Every time we get into the slicing time we generate a new traj
#                 print "   ----SLICING----"
#                 print "Adding traj with " + str(len(rt.joint_trajectory.points)) + " points"
#                 slice_counter += 1
#                 print "This is slice #" + str(slice_counter)
                sliced_trajectories.append(rt)
                rt = RobotTrajectory() # Create a new one
                rt.joint_trajectory.joint_names = robot_trajectory.joint_trajectory.joint_names
                next_slice_time = point.time_from_start + rospy.Duration(slice_time)

            new_time = point.time_from_start.to_sec() % slice_time # We want times from 0.0 to slice_time
            new_point = copy.deepcopy(point)
            new_point.time_from_start = rospy.Duration(new_time)
            rt.joint_trajectory.points.append(new_point)
            print "Old time: " + str(point.time_from_start.to_sec()) + " new time: " + str(new_point.time_from_start.to_sec())
        sliced_trajectories.append(rt) # Append last trajectory
        
        rospy.loginfo("We sliced the trajectory of " + str(len(robot_trajectory.joint_trajectory.points)) + " points in "\
                      + str(len(sliced_trajectories)) + " trajectories of max " + str(len(sliced_trajectories[0].joint_trajectory.points)) + " points" )
        last_sent_time = None
        initial_time = rospy.Time.now()
        for id, traj in enumerate(sliced_trajectories):
            if self.checkTrajectoryValidity(traj):
                ektr = createExecuteKnownTrajectoryRequest(traj, False)
                if id > 0:
                    # Wait for the trajectory to be executed until the point we can send the next one...
                    rospy.loginfo("Waiting for last trajectory to achieve the time where sending the next trajectory will connect them nicely")
                    pre_wait_time = rospy.Time.now()
                    rospy.loginfo("  [] Before wait time: " + str((rospy.Time.now() - initial_time).to_sec()))
                    while rospy.Time.now() - last_sent_time < rospy.Duration(slice_time - call_delay):
                        rospy.sleep(0.001)
                    after_wait_time = rospy.Time.now()
                    rospy.loginfo("  [] After wait time: " + str((rospy.Time.now() - initial_time).to_sec()))
                    rospy.loginfo("We waited for " + str((after_wait_time - pre_wait_time).to_sec()))
                rospy.loginfo("Sending trajectory... #" + str(id) + "/" + str(len(sliced_trajectories)))
                if last_sent_time != None:
                    rospy.loginfo("We took from last sent time to now: " + str((rospy.Time.now() -last_sent_time).to_sec()))
                last_sent_time = rospy.Time.now()
                trajectory_executed = False
                while not trajectory_executed:
                    rospy.loginfo("  [] Sending traj at time: " + str((rospy.Time.now() - initial_time).to_sec()))
                    answer = self.execute_known_traj_service.call(ektr)
                    rospy.loginfo("  [] Service call done at time: " + str((rospy.Time.now() - initial_time).to_sec()))
                    #ExecuteKnownTrajectoryResponse.error_code.val
                    if answer.error_code.val == MoveItErrorCodes.SUCCESS:
                        trajectory_executed = True
                    else:
                        #print "not sleeping"
                        rospy.sleep(0.01)
                    rospy.logwarn("Answer of execute service: " + str(answer))
                
                # We actually should rewrite all the times to match with the necessary offset...
                # Let's try this first!

                rospy.loginfo("Sent trajectory!")
            else:
                rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
                break


        
    def getCurrentJointsPose(self, joint_names):
        """Given a set of joints (or group?) get the list of positions of these joints"""
        # Prepare subscriber
        # Creating a subscriber to joint states
        current_joint_state = rospy.wait_for_message(DEFAULT_JOINT_STATES, JointState)
        joint_names, values = self.getNamesAndMsgList(joint_names, current_joint_state)
        return values

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
    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    ge = gestureExecution()
    
