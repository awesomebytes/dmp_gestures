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

from pal_interaction_msgs.msg import SoundAction, SoundActionGoal, SoundGoal

def createSoundGoal(sentence):
    sag = SoundActionGoal()
    sag.goal.text = sentence
    return sag

if __name__ == '__main__':
    rospy.init_node("step_by_step_learn_from_play_motion")
    rospy.loginfo("Preparing recorder...")
    sound_pub = rospy.Publisher('/sound/goal', SoundActionGoal)
    rospy.sleep(0.3)
    sound_pub.publish(createSoundGoal("Getting ready"))
    # Start motion recorder
    rfpm = RecordFromPlayMotion()
    rospy.sleep(10.0)
    # TODO: give a list of the available motions getting them from param server
    #motion_name = str(raw_input('Write the name of the motion to play-and-record: '))
    motion_name = "wave"
    rospy.loginfo("Got as input: " + str(motion_name))
    sound_pub.publish(createSoundGoal("I can learn from previously created by hand movements"))
    rospy.sleep(4.0)
    sound_pub.publish(createSoundGoal("I'm going to play the motion " + motion_name + " and learn from it"))
    # Check if there exist the files related to this motion already to not overwrite
    if isfile(motion_name + ".bag"):
        rospy.loginfo("There is already a rosbag called " + motion_name + ".bag, aborting.")
        exit(0)
    
    motiondata = rfpm.play_and_record(motion_name, bag_name = motion_name)
    rospy.loginfo("Motion played and recorded, there should be a bag named: " + motion_name + ".bag")
    rospy.loginfo("The data for this bag is: " + str(motiondata))
    # Looks like:
#      {'joints': ['head_2_joint'],
#      'rosbag_name': 'yes.bag',
#      'motion_name': 'yes'}
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
    
    sound_pub.publish(createSoundGoal("Now I'm going to learn from the trajectory recorded, I'll need some seconds"))

    
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
    
    sound_pub.publish(createSoundGoal("Ok, I learnt the gesture, now I'm going to play my version from my current position"))
    
    init_time = time.time()
    plan = gG.getPlan(curr_joints_pose, gesture_dict["final_pose"], tau=gesture_dict["duration"], dt=0.02)
    rospy.loginfo("It took " + str(time.time() - init_time ) + "s to get the plan.")
    
    rospy.loginfo("Now we display the plan in Rviz. You need a MotionPlanning plugin set (comes with MoveIt!) with Planned Path activated. State display time is recommended to be at REALTIME.")
    traj = ge.displayTrajFromPlan(plan, joint_names, curr_joints_pose)
    pub.publish(traj)
    
    rospy.loginfo("Now we can generate the plot for the joints. Close the window to continue.")
   
#     plot = dp.planToPlot(plan, joint_names)
#     plot.show()
    
    rospy.loginfo("From this plan we need to form a robot trajectory, let's do that.")
    robot_traj = ge.robotTrajectoryFromPlan(plan, joint_names)
    
    rospy.loginfo("And finally we can send the trajectory to the controllers.")
    rospy.loginfo("If the trajectory has some self collision problem it won't be executed.")
    ge.sendTrajectory(robot_traj, True)
    
    rospy.loginfo("End of demo.")
    