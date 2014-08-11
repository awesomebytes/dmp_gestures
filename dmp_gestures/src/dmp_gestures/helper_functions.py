#!/usr/bin/python
"""
Created on 19/05/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

This file intends to contain helper functions.
"""

import rospy

from moveit_msgs.msg import MoveItErrorCodes
from actionlib import GoalStatus

# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

goal_status_dict = {}
for name in GoalStatus.__dict__.keys():
    if not name[:1] == '_':
        code = GoalStatus.__dict__[name]
        goal_status_dict[code] = name

if __name__ == '__main__':
    rospy.init_node("test_helper_functions")
    rospy.loginfo("Initializing helper_functions test.")
