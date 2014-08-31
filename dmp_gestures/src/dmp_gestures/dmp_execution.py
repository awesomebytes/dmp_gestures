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
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse, GetPositionIK
from moveit_msgs.msg._MoveItErrorCodes import MoveItErrorCodes
import copy
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf
from helper_functions import moveit_error_dict

DEFAULT_JOINT_STATES = '/joint_states'
EXECUTE_KNOWN_TRAJ_SRV = '/execute_kinematic_path'
DEFAULT_IK_SERVICE = "/compute_ik"

DEBUG_MODE =  True

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
        #TODO: make ik_service_name a param to load from a yaml
        self.ik_service_name = DEFAULT_IK_SERVICE
        # Get a ServiceProxy for the IK service
        rospy.loginfo("Waiting for service '" + self.ik_service_name + "'...")
        rospy.wait_for_service(self.ik_service_name)
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, GetPositionIK)
        rospy.loginfo("Successful connection  to '" + self.ik_service_name + "'.")

# TODO: Get rid of these list and load them from play_motion params as done in dmp_training.py
        self.joint_list = ['torso_1_joint', 'torso_2_joint',
                           'head_1_joint', 'head_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint', 
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']


        if DEBUG_MODE:
            self.pub_ok_markers = rospy.Publisher('ik_ok_marker_list', MarkerArray, latch=True)
            self.ok_markers = MarkerArray()
        
            self.pub_fail_markers = rospy.Publisher('ik_fail_marker_list', MarkerArray, latch=True)
            self.fail_markers = MarkerArray()
            self.markers_id = 5    
        
    def robotTrajectoryFromPlanPoseBased(self, plan, groups, downsample_freq=None):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory doing IK calls for the PoseStampeds
        provided so we can visualize and execute the motion"""
        rt = RobotTrajectory()
        jt = JointTrajectory()
        #jt.joint_names
        jt.points
        # Magic code goes here.
        fjtg = self.computeJointTrajFromCartesian(plan.plan.points, plan.plan.times, downsample_freq=downsample_freq)
        rt.joint_trajectory = fjtg.trajectory
        #plan.plan.times should be the times used 
        if len(rt.joint_trajectory.points) != len(plan.plan.times):
            rospy.logerr("joint trajectory point does not have same length than planned times, this means there are IKs that failed")
            rospy.logerr("points: " + str(len(rt.joint_trajectory.points)) + " times: " + str(len(plan.plan.times)))
            
#         for point, time in zip(rt.joint_trajectory.points, plan.plan.times):
#             # Probably need to allocate it again here...
#             point.time_from_start = rospy.Duration(time)
        return rt
        
    def robotTrajectoryFromPlanPoseBasedDownSampledAndWithDMP(self, plan, groups):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory doing IK calls for the PoseStampeds
        provided so we can visualize and execute the motion"""
        rt = RobotTrajectory()
        jt = JointTrajectory()
        #jt.joint_names
        jt.points
        # Magic code goes here.
        fjtg = self.computeJointTrajFromCartesian(plan.plan.points, plan.plan.times)
        rt.joint_trajectory = fjtg.trajectory
        #plan.plan.times should be the times used 
        if len(rt.joint_trajectory.points) != len(plan.plan.times):
            rospy.logerr("joint trajectory point does not have same length than planned times, this means there are IKs that failed")
            rospy.logerr("points: " + str(len(rt.joint_trajectory.points)) + " times: " + str(len(plan.plan.times)))
            
#         for point, time in zip(rt.joint_trajectory.points, plan.plan.times):
#             # Probably need to allocate it again here...
#             point.time_from_start = rospy.Duration(time)
        return rt
        
        
    def robotTrajectoryFromPlan(self, plan, joint_names):
        """Given a dmp plan (GetDMPPlanResponse) create a RobotTrajectory to be able to visualize what it consists and also
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

    def displayTrajFromRobotTraj(self, robot_traj):
        """Given a robot trajectory return a displaytrajectory with it"""
        dt = DisplayTrajectory()
        dt.trajectory.append(robot_traj)
        dt.trajectory_start.joint_state.name = robot_traj.joint_trajectory.joint_names
        dt.trajectory_start.joint_state.position = robot_traj.joint_trajectory.points[0].positions
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
            return True
        else:
            rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
            return False

    def sendTrajectoryAndRecordBag(self, robot_trajectory, motion_name, joints, bag_name, wait_for_execution=False):
        """Given a RobotTrajectory send it to the controllers firstly checking it's trajectory validity"""
        from dmp_training import LearnFromJointState
        lfjs = LearnFromJointState()
        
        if self.checkTrajectoryValidity(robot_trajectory):
            ektr = createExecuteKnownTrajectoryRequest(robot_trajectory, wait_for_execution)
            rospy.loginfo("Sending trajectory...")
            time_init = time.time()
            ros_time_init = rospy.Time.now()
            self.execute_known_traj_service.call(ektr)
            lfjs.start_learn(motion_name, joints, bag_name)
            # Wait until completion of the trajectory.... kind of the duration of the traj
            # Get last trajectory time
            trajectory_final_time = robot_trajectory.joint_trajectory.points[-1].time_from_start
            rospy.sleep(trajectory_final_time)
            lfjs.stop_learn()
            ros_time_fin = rospy.Time.now()
            time_fin = time.time()
            ros_time_diff = ros_time_fin - ros_time_init
            
            rospy.loginfo("Finished trajectory! The service call took (realtime): " + str(time_fin - time_init) + " (ros time): " + str(ros_time_diff.to_sec()) + "s " + str(ros_time_diff.to_nsec()) + "ns")
            return True
        else:
            rospy.logerr("Trajectory in collision at some point, won't be sent to controllers.")
            return False



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
    
    def getCurrentEndEffectorPose(self, tf_frame):
        """Given a set of joints (or group?) get the list of positions of these joints"""
        transformer = tf.TransformListener()
        rospy.sleep(0.9)
        if not transformer.frameExists(tf_frame):
            print "Frame " + str(tf_frame) + " does not exist."
            exit(0)
        transformer.waitForTransform(tf_frame, 'base_link', rospy.Time(), rospy.Duration(20.0))
        ps = PoseStamped()
        pos, quat = transformer.lookupTransform('base_link', tf_frame, rospy.Time())
        #rospy.loginfo("pos: " + str(pos) + " quat: " + str(quat))
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'base_link'
        ps.pose.position = Point(*pos)
        ps.pose.orientation = Quaternion(*quat)
        return ps

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

    def getIkPose(self, pose, group="right_arm", previous_state=None):
        """Get IK of the pose specified, for the group specified, optionally using
        the robot_state of previous_state (if not, current robot state will be requested) """
        # point point to test if there is ik
        # returns the answer of the service
        rqst = GetPositionIKRequest()
        rqst.ik_request.avoid_collisions = True
        rqst.ik_request.group_name = group
        rqst.ik_request.pose_stamped.header = Header(stamp=rospy.Time.now())
        rqst.ik_request.pose_stamped.header.frame_id = 'base_link'


        # Set point to check IK for
        rqst.ik_request.pose_stamped.pose.position = pose.position
        rqst.ik_request.pose_stamped.pose.orientation = pose.orientation
        
        if previous_state == None:
            current_joint_state = rospy.wait_for_message(DEFAULT_JOINT_STATES, JointState)
            cs = RobotState()
            cs.joint_state = current_joint_state
            #cs = self.r_commander.get_current_state() # old code
            rqst.ik_request.robot_state = cs
        else:
            rqst.ik_request.robot_state = previous_state

        ik_answer = GetPositionIKResponse()
        if DEBUG_MODE:
            timeStart = rospy.Time.now()
        ik_answer = self.ik_serv.call(rqst)
        
        if DEBUG_MODE:
            durationCall= rospy.Time.now() - timeStart
            rospy.loginfo("Call took: " + str(durationCall.to_sec()) + "s")
        
        return ik_answer   
        
    def computeJointTrajFromCartesian(self, points, times, arm="right_arm_torso", downsample_freq=None):
        #fjt_goal = FollowJointTrajectoryGoal()
        poselist = []
        for point in points:
            qt = quaternion_from_euler(point.positions[3], point.positions[4], point.positions[5])
            pose = Pose(Point(point.positions[0], point.positions[1], point.positions[2]),
                        Quaternion(*qt.tolist()))
            poselist.append(pose)
        fjt_goal = self.computeIKsPose(poselist, times, arm, downsample_freq)
        
        return fjt_goal
        
        
    def computeIKsPose(self, poselist, times, arm="right_arm", downsample_freq=None):
        """Giving a poselist (list of Pose) and times for every point compute it's iks and add it's times
        if a value is given to downsample_freq then we will only compute IKs and so for the downsampled
        rate of poses"""
        rospy.loginfo("Computing " + str(len(poselist)) + " IKs" )
        fjt_goal = FollowJointTrajectoryGoal()

        if arm == 'right_arm_torso':
            fjt_goal.trajectory.joint_names = self.right_arm_torso
        elif arm == 'left_arm_torso':
            fjt_goal.trajectory.joint_names = self.left_arm_torso
        elif arm == 'right_arm':
            fjt_goal.trajectory.joint_names = self.right_arm
        elif arm == 'left_arm':
            fjt_goal.trajectory.joint_names = self.left_arm
        # TODO: add other options
        
        ik_answer = None
        last_succesfull_ik_answer = None
        if downsample_freq != None:
            num_poses = 0
            num_downsampled_poses = 0
        for pose, time in zip(poselist, times):
            if downsample_freq != None:
                num_poses += 1
                if num_poses % downsample_freq == 0:
                    num_downsampled_poses += 1
                else: # if we are downsampling dont do IK calls if it's not one of the samples we want
                    continue
            if last_succesfull_ik_answer != None:
                ik_answer = self.getIkPose(pose,"right_arm", previous_state=last_succesfull_ik_answer.solution)
            else:
                ik_answer = self.getIkPose(pose)
            if DEBUG_MODE:
                rospy.loginfo("Got error_code: " + str(ik_answer.error_code.val) + " which means: " + moveit_error_dict[ik_answer.error_code.val])
            if moveit_error_dict[ik_answer.error_code.val] == 'SUCCESS':
                # We should check if the solution is very far away from joint config
                # if so.. try again... being a generic solution i dont know how to manage this
                last_succesfull_ik_answer = ik_answer
                if DEBUG_MODE:
                    arrow = self.createArrowMarker(pose, ColorRGBA(0,1,0,1))
                    self.ok_markers.markers.append(arrow)
                jtp = JointTrajectoryPoint()
                #ik_answer = GetConstraintAwarePositionIKResponse()
                # sort positions and add only the ones of the joints we are interested in
                positions = self.sortOutJointList(fjt_goal.trajectory.joint_names, ik_answer.solution.joint_state)
                jtp.positions = positions
                jtp.time_from_start = rospy.Duration(time)
                fjt_goal.trajectory.points.append(jtp)
                if DEBUG_MODE:
                    self.pub_ok_markers.publish(self.ok_markers)
                
            else:
                if DEBUG_MODE:
                    arrow = self.createArrowMarker(pose, ColorRGBA(1,0,0,1))
                    self.fail_markers.markers.append(arrow)
                    self.pub_fail_markers.publish(self.fail_markers)
                    # Loop for a while to check if we get a solution on further checks?
                    
        if downsample_freq != None:
            rospy.loginfo("From " + str(num_poses) + " points we downsampled to " + str(num_downsampled_poses) + " and fjt_goal.trajectory.points has " + str(fjt_goal.trajectory.points) + " points")
        return fjt_goal
           
    def sortOutJointList(self, joint_name_list, joint_state):
        """ Get only the joints we are interested in and it's values and return it in
        joint_state.name and joint_state.points format"""
        if DEBUG_MODE:
            rospy.loginfo("Sorting jointlist...")
        list_to_iterate = joint_name_list      
        curr_j_s = joint_state
        ids_list = []
        position_list = []
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            position_list.append(curr_j_s.position[idx_in_message])
        return position_list

    def adaptTimesAndVelocitiesOfMsg(self, trajectory, plan, desired_final_time):
        """Adapt the times and velocities of the message for the controller
        from the times computed in the DMP and velocities 0.0, controller will take care of it"""
        rospy.loginfo("Adapting times and velocities...")
        traj = trajectory #FollowJointTrajectoryGoal()
        p = plan #GetDMPPlanResponse()
        # we should have the same number of points on each, NOPE, as we can have points that IK fails
#         if len(traj.trajectory.points) != len(p.plan.points):
#             rospy.logerr("Oops, something went wrong, different number of points")
#             rospy.logerr("generated trajectory: " + str(len(traj.trajectory.points)) + " plan: " + str(len(p.plan.points)))
#             exit(0)
        
        point = JointTrajectoryPoint()
        counter = 0
        for point in traj.trajectory.points:
            #rospy.loginfo("Step " + str(counter) + " / " + str(len(traj.trajectory.points)))
            counter += 1
            point.velocities.extend(len(point.positions) * [0.0]) # adding 0.0 as speeds
            point.time_from_start = rospy.Duration( counter * (desired_final_time / len(traj.trajectory.points)) ) 
        return traj
                
    def publish_markers(self, time=10.0):
        init_time = rospy.Time.now()
        time_done = False
        while not time_done:
            self.pub_ok_markers.publish(self.ok_markers)
            self.pub_fail_markers.publish(self.fail_markers)
            rospy.sleep(0.1)
            if rospy.Time.now() - init_time > rospy.Duration(time):
                time_done = True
        
    def createArrowMarker(self, pose, color):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        general_scale = 0.01
        marker.scale.x = general_scale
        marker.scale.y = general_scale / 3.0
        marker.scale.z = general_scale / 10.0
        marker.color = color
        marker.pose.orientation = pose.orientation
        marker.pose.position = pose.position
        marker.id = self.markers_id
        self.markers_id += 1
        return marker


if __name__ == '__main__':
    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    ge = gestureExecution()
    
