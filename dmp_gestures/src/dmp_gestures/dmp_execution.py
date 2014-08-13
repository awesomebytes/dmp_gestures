#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains execution related classes.
"""

import rospy
from dmp.srv import GetDMPPlan, GetDMPPlanRequest, GetDMPPlanResponse,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class gestureExecution():
    
    def __init__(self):
        rospy.loginfo("Initializing gestureExecution")
        
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
            rospy.logwarn("jtp is: " + str(jtp))
            rt.joint_trajectory.points.append(jtp)
        return rt
    
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


if __name__ == '__main__':
    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
    ge = gestureExecution()
    
