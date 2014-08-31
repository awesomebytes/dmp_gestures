#!/usr/bin/python
"""
Created on 19/08/14

@author: Sammy Pfeiffer
@email: sammypfeiffer@gmail.com


"""
# System dependencies
from os.path import isfile
import time
import copy
# ROS dependencies
import rospy
# Message dependencies
from moveit_msgs.msg import DisplayTrajectory
# This example dependencies
from dmp_training import RecordFromPlayMotion
from dmp_generation import gestureGeneration
from dmp_execution import gestureExecution
from dmp_generation import dmpPlanTrajectoryPlotter
from plot_rosbag_trajectory import rosbagTrajectoryPlotter

if __name__ == '__main__':
    rospy.init_node("step_by_step_learn_from_play_motion")
    rospy.loginfo("Preparing recorder...")
    # Start motion recorder
    rfpm = RecordFromPlayMotion()
    joints_right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                            'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                            'arm_right_7_joint']
    # TODO: give a list of the available motions getting them from param server
    motion_name = str(raw_input('Write the name of the motion to play-and-record: '))
    rospy.loginfo("Got as input: " + str(motion_name))
    # Check if there exist the files related to this motion already to not overwrite
    if isfile(motion_name + ".bag"):
        rospy.loginfo("There is already a rosbag called " + motion_name + ".bag, aborting.")
        exit(0)
    
    motiondata = rfpm.play_and_record(motion_name, joints=joints_right_arm, bag_name = motion_name)
    rospy.loginfo("Motion played and recorded, there should be a bag named: " + motion_name + ".bag")
    rospy.loginfo("The data for this bag is: " + str(motiondata))
    # Looks like:
#      {'joints': ['head_2_joint'],
#      'rosbag_name': 'yes.bag',
#      'motion_name': 'yes'}

    # Analyze only one arm
    rtp = rosbagTrajectoryPlotter()

    rospy.loginfo("========== PLOTTING RIGHT ARM FROM THE RECORDED BAG ========")
    plot = rtp.readBagAndCreatePlot(motion_name + ".bag", joints_right_arm)
    plot.show()
    
    rospy.loginfo("========== PLOTTING DOWNSAMPLED TRAJ =======")
    plot = rtp.readAndDownSampleBag(motion_name + ".bag", joints_right_arm, frequency_to_downsample=15)
    plot.show()
    
    
    joint_names = motiondata['joints']
    rospy.loginfo("Preparing other nodes...")
    # Get publisher for displaying generated trajectory TODO: add this to the class itself
    pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory)
    # Start gesture generator
    gG = gestureGeneration()
    # Start gesture executor 
    ge = gestureExecution()
    # Start plotter
    dp = dmpPlanTrajectoryPlotter()
    rospy.loginfo("Nodes initialized.")
    
    rospy.loginfo("Now we will generate a DMP from this bag.")
    #gesture_dict = gG.loadGestureFromBagJointStates(motion_name +".bag", joint_names)
    gesture_dict = gG.loadGestureFromBagJointStatesAndRemoveJerkiness(motion_name +".bag", joint_names)
    #gesture_dict = gG.loadGestureFromBagJointStatesAndDownsample(motion_name +".bag", joint_names)
    
    copy_gest_dict = copy.deepcopy(gesture_dict)
    del copy_gest_dict['computed_dmp']
    rospy.loginfo("Now we got a DMP and a gesture dictionary containing it, looks like (omitting computed_dmp): " + str(copy_gest_dict))
    
    
    rospy.loginfo("We want to get a plan for this motion from our current position so first we grab the current pose of the robot.")
    curr_joints_pose = ge.getCurrentJointsPose(joint_names)
    rospy.loginfo("Current joints pose:\n" + str(curr_joints_pose) + "\n for joints: " + str(joint_names))
    
    rospy.loginfo("Let's get a plan for our motion from the current pose.")
    rospy.loginfo("We request the plan from the current joints pose, to the final pose which was recorded in the example.")
    rospy.loginfo("The plan should take as long as the learnt gesture: " + str(gesture_dict["duration"]) + "s and we compute ")
    rospy.loginfo("points every dt = 0.1s.")
    init_time = time.time()
    plan = gG.getPlan(curr_joints_pose, gesture_dict["final_pose"], tau=gesture_dict["duration"], dt=0.02)
    rospy.loginfo("It took " + str(time.time() - init_time ) + "s to get the plan.")
    
    rospy.loginfo("Now we display the plan in Rviz. You need a MotionPlanning plugin set (comes with MoveIt!) with Planned Path activated. State display time is recommended to be at REALTIME.")
    traj = ge.displayTrajFromPlan(plan, joint_names, curr_joints_pose)
    pub.publish(traj)
    
    rospy.loginfo("Now we can generate the plot for the joints. Close the window to continue.")
   
    rospy.loginfo("============ PLOTTING RIGHT ARM PLAN ======== ")
    plot = dp.planToPlot(plan, joints_right_arm)
    plot.show()
    
    rospy.loginfo("From this plan we need to form a robot trajectory, let's do that.")
    robot_traj = ge.robotTrajectoryFromPlan(plan, joints_right_arm)
    
    rospy.loginfo("And finally we can send the trajectory to the controllers.")
    rospy.loginfo("If the trajectory has some self collision problem it won't be executed.")
    ge.sendTrajectoryAndRecordBag(robot_traj, motion_name, joints_right_arm, motion_name + "_executed.bag", wait_for_execution=False)
    #ge.sendTrajectory(robot_traj, True)
    
    rospy.loginfo("=========== PLOTTING EXECUTED TRAJECTORY FROM DMP =======")
    bag_name = "crowd_salute_executed.bag.bag"  #'crowd_salute_jerky.bag'
    plot = rtp.readBagAndCreatePlot(bag_name, joints_right_arm)
    plot.show()
    
    rospy.loginfo("End of demo.")
    