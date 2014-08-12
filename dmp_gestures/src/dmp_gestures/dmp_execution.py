#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file contains execution related classes.
"""

import rospy
from dmp.srv import GetDMPPlan, GetDMPPlanRequest,LearnDMPFromDemo, LearnDMPFromDemoRequest, SetActiveDMP, SetActiveDMPRequest


class gestureExecution():
    
    def __init__(self):
        rospy.loginfo("Initializing gestureExecution")




if __name__ == '__main__':
    rospy.init_node("test_execution_classes")
    rospy.loginfo("Initializing dmp_execution test.")
