#!/usr/bin/python
"""
Created on 14/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com

"""

import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, DisplayRobotState, Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFKResponse
from kinematics_interface import ForwardKinematics, InverseKinematics
import copy
import random

#from moveit_commander import PlanningSceneInterface, MoveGroupCommander #, roscpp_initialize, roscpp_shutdown
# from moveit_ros_planning_interface import _moveit_move_group_interface, _moveit_planning_scene_interface, _moveit_robot_interface
# from moveit_ros_planning_interface._moveit_planning_scene_interface import PlanningSceneInterface
# from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface
# from moveit_ros_planning_interface._moveit_move_group_interface import MoveGroup
# from moveit_commander import MoveGroupCommandInterpreter
# MoveGroupCommandInterpreter.

if __name__ == '__main__':
    rospy.init_node("test_kinematics_class")
    rospy.loginfo("Initializing forward kinematics test.")
    fk = ForwardKinematics()
    rospy.loginfo("Current FK:")
    rospy.loginfo(str(fk.getCurrentFK('arm_left_7_link')))
    
    current_joint_states = rospy.wait_for_message('/joint_states', JointState)
    current_joint_states_modif = copy.deepcopy(current_joint_states)
    current_joint_states_modif.position = [2.0] * len(current_joint_states_modif.position)
    rs_pub = rospy.Publisher('fk_test_robot_state', DisplayRobotState)
    rospy.sleep(0.3) # let it initialize...
    rospy.loginfo("Setting random poses and watching output")
    # Check for self collisions
#     scene = PlanningSceneInterface()
#     mg = MoveGroup()
    ik = InverseKinematics()


    for i in range(10):
        rospy.loginfo("Random pose: " + str(i))
        random_js = copy.deepcopy(current_joint_states)
        positions_list = []
        for id, item in enumerate(current_joint_states_modif.position):
            positions_list.append(random.random() * 3.14)
        random_js.position = positions_list
#         c = Constraints()
#         jc = JointConstraint()
#         jc.joint_name = 'arm_right_1_link'
#         c.joint_constraints.append()
        result = fk.getFK('arm_right_7_link', current_joint_states.name, random_js.position, 'base_link')
        rospy.loginfo("Result of a disastrous robot pose is: " + str(result))

        rs = RobotState()
        rs.joint_state = random_js
        drs = DisplayRobotState()
        drs.state = rs
        rs_pub.publish(drs)
        rospy.loginfo("Published robot state")
        rospy.loginfo("Asking for IK with collision avoidance for the pose given")
        print "Result is:"
        print result
        print "And result.pose_stamped is:"
        print result.pose_stamped
        print "with result.pose_stamped[0]"
        print result.pose_stamped[0]
        resultik = ik.getIK("right_arm", "right_arm_7_link", result.pose_stamped[0], True, 0, rs)
        rospy.loginfo(str(resultik))
        rospy.sleep(1.0)

    
#     rospy.loginfo("isJointConfigInCollision with all left arm at 0.0 (should be False):")
#     rospy.loginfo( str(fk.isJointConfigInCollision('arm_left_7_link', 
#                                 ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
#                                  'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
#                                  'arm_left_7_joint'],
#                                  [0.0, 0.0, 0.0,
#                                   0.0, 0.0, 0.0,
#                                   0.0]) ))
#     rospy.loginfo("isJointConfigInCollision with shoulder pointing inwards (should be True):")
#     rospy.loginfo( str(fk.isJointConfigInCollision('arm_left_7_link', 
#                                 ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
#                                  'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
#                                  'arm_left_7_joint'],
#                                  [-2.0, 0.0, 0.0,
#                                   0.0, 0.0, 0.0,
#                                   0.0]) ))
    
    
    fk.closeFK()