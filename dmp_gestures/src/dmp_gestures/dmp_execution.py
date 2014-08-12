#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains execution related classes.
"""

import rospy
from dmp.srv import GetDMPPlan, GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory


class gestureExecution():
    
    def __init__(self):
        rospy.loginfo("Initializing gestureExecution")
        
    def robotTrajectoryFromPlan(self, plan, joint_names):
        """Given a dmp plan create a RobotState to be able to visualize what it consists and also
        to be bale to send it to execution"""
        rt = RobotTrajectory()
        rt.joint_trajectory.joint_names = joint_names
        rt.joint_trajectory.points = plan.points
    
    def displayTrajFromPlan(self, plan, joint_names, initial_state):
        """Given a plan, joint_names list and an initial_state as RobotState
        Create a displayTraj message"""
        dt = DisplayTrajectory()
        dt.trajectory = self.robotTrajectoryFromPlan(plan, joint_names)
        dt.trajectory_start = initial_state
        dt.model_id = "something"




if __name__ == '__main__':
    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
