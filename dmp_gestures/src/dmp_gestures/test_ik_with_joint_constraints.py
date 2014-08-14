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
from kinematics_interface import ForwardKinematics, InverseKinematics, StateValidity
import copy
import random
from moveit_msgs.srv._GetPositionIK import GetPositionIKResponse


if __name__ == '__main__':
    rospy.init_node("test_kinematics_class_ik_stuff")
    rospy.loginfo("Initializing forward kinematics test.")
    fk = ForwardKinematics()

    rospy.loginfo("Getting a current joint states message")
    current_joint_states = rospy.wait_for_message('/joint_states', JointState)
    current_joint_states_modif = copy.deepcopy(current_joint_states)
    current_joint_states_modif.position = [2.0] * len(current_joint_states_modif.position)
    rs_pub = rospy.Publisher('fk_test_robot_state', DisplayRobotState)
    rospy.sleep(0.3) # let it initialize...

    ik = InverseKinematics()

    correct_js = copy.deepcopy(current_joint_states)


    result = fk.getFK('arm_right_7_link', current_joint_states.name, correct_js.position, 'base_link')
    rospy.loginfo("Result of current robot pose FK is: " + str(result))

    rs = RobotState()
    rs.joint_state = correct_js
    drs = DisplayRobotState()
    drs.state = rs
    rs_pub.publish(drs)
    rospy.loginfo("Published current robot state")
    rospy.sleep(2.0)

    c = Constraints()
    jc = JointConstraint()
    jc.joint_name = 'arm_right_1_link'
    jc.position = 0.0
    jc.tolerance_above = 0.00001
    jc.tolerance_below = 0.00001
    jc.weight = 1.0
    c.joint_constraints.append(jc)
    
    rospy.loginfo("Result without constraints:")
    resultik = ik.getIK("right_arm", "right_arm_7_link", result.pose_stamped[0], False, 0, rs)
    rospy.loginfo(str(resultik))
    
    rs = RobotState()
    rs.joint_state = resultik.solution.joint_state
    drs = DisplayRobotState()
    drs.state = rs
    rs_pub.publish(drs)
    rospy.loginfo("Published solution with False collision avoidance robot state")
    rospy.sleep(5.0)
    
    
    rospy.loginfo("Result WITH constraints:")
    rospy.loginfo("Constraints being: " + str(c))
    resultik = ik.getIK("right_arm", "right_arm_7_link", result.pose_stamped[0], False, 0, rs, c)
    rospy.loginfo(str(resultik))
    rospy.sleep(1.0)

    rs = RobotState()
    rs.joint_state = resultik.solution.joint_state
    drs = DisplayRobotState()
    drs.state = rs
    rs_pub.publish(drs)
    rospy.loginfo("Published solution with True collision avoidance robot state")
    rospy.sleep(5.0)
    
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