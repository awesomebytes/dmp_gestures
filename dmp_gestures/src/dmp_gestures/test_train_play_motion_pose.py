#!/usr/bin/python
"""
Created on 28/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com


"""
import rospy
from dmp_training import RecordPoseStampedFromPlayMotion
from dmp_generation import gestureGeneration
from dmp_execution import gestureExecution
from dmp_generation import dmpPlanTrajectoryPlotter
from moveit_msgs.msg import DisplayTrajectory
import time
import copy
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
    rospy.init_node("test_train_from_posestampd_")
    rospy.loginfo("Initializing test.")
    # Get publisher for displaying generated trajectory TODO: add this to the class itself
    pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory)
    motion = "wave"
    gesture_name =  motion
    rpsfpm = RecordPoseStampedFromPlayMotion()
    motiondata = rpsfpm.play_and_record(gesture_name, groups=['right_arm'],bag_name = gesture_name)
    rospy.loginfo("Motion dict is: " + str(motiondata))
    
    gG = gestureGeneration()
    rospy.loginfo("loading gesture from bag end effector")
    gesture_dict = gG.loadGestureFromBagEndEffector(motiondata['rosbag_name'], ['/tf_to_ps'], ['right_arm'])
    rospy.loginfo("Loaded!")
    rospy.loginfo("gest dict: " + str(gesture_dict))
    
    gE = gestureExecution()
    joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']

    copy_gest_dict = copy.deepcopy(gesture_dict)
    del copy_gest_dict['computed_dmp']
    rospy.loginfo("Now we got a DMP and a gesture dictionary containing it, looks like (omitting computed_dmp): " + str(copy_gest_dict))
    
    rospy.loginfo("We want to get a plan for this motion from our current position so first we grab the current pose of the robot.")
    # Get current end effector position to start off there!
    curr_eef_pose = gE.getCurrentEndEffectorPose("hand_right_grasping_frame")
    roll, pitch, yaw = euler_from_quaternion([curr_eef_pose.pose.orientation.x, curr_eef_pose.pose.orientation.y, curr_eef_pose.pose.orientation.z, curr_eef_pose.pose.orientation.w])
    curr_eef_pose_list = [curr_eef_pose.pose.position.x, curr_eef_pose.pose.position.y, curr_eef_pose.pose.position.z, roll, pitch, yaw]

    rospy.loginfo("Let's get a plan for our motion from the current pose.")
    rospy.loginfo("We request the plan from the current joints pose, to the final pose which was recorded in the example.")
    rospy.loginfo("The plan should take as long as the learnt gesture: " + str(gesture_dict["duration"]) + "s and we compute ")
    rospy.loginfo("points every dt = 0.1s.")
    init_time = time.time()
    plan = gG.getPlan(curr_eef_pose_list, gesture_dict["final_pose"], tau=gesture_dict["duration"], dt=0.02)
    rospy.loginfo("It took " + str(time.time() - init_time ) + "s to get the plan.")
    

    rt = gE.robotTrajectoryFromPlanPoseBased(plan, ['right_arm'], downsample_freq=10)
    
    rospy.loginfo("Now we display the plan in Rviz. You need a MotionPlanning plugin set (comes with MoveIt!) with Planned Path activated. State display time is recommended to be at REALTIME.")
    traj = gE.displayTrajFromRobotTraj(rt)
    pub.publish(traj)

    rospy.loginfo("Publishing markers")
    gE.publish_markers(3.0) # publish for 3s
    
#     rospy.loginfo("Now we can generate the plot for the joints. Close the window to continue.")
#     dp = dmpPlanTrajectoryPlotter()
#     plot = dp.planToPlot(plan, joint_names)
#     plot.show()
    
#     rospy.loginfo("From this plan we need to form a robot trajectory, let's do that.")
#     robot_traj = gE.robotTrajectoryFromPlan(plan, joint_names)
    
    rospy.loginfo("And finally we can send the trajectory to the controllers.")
    rospy.loginfo("If the trajectory has some self collision problem it won't be executed.")
    gE.sendTrajectory(rt, True)
    
    rospy.loginfo("End of demo.")