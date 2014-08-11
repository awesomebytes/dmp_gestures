#!/usr/bin/python

import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult
PLAY_MOTION_AS = "/play_motion"
from helper_functions import goal_status_dict

def test_done_cb():
    print "We got a done!"


if __name__ == '__main__':
    rospy.init_node("test_donecb")
    rospy.loginfo("Connecting to AS: '" + PLAY_MOTION_AS + "'")
    play_motion_as = SimpleActionClient(PLAY_MOTION_AS, PlayMotionAction)
    play_motion_as.wait_for_server()
    rospy.loginfo("Connected.")
    play_motion_as.done_cb = test_done_cb # This is never called
    pm_goal = PlayMotionGoal("home", False, 0)
    play_motion_as.send_goal(pm_goal)
    rospy.loginfo("Sent goal.")
    #play_motion_as.wait_for_result() # I thought this was necessary to actually start the goal, but it's not
    done = False
    while not done:
        state = play_motion_as.get_state()
        rospy.loginfo("State: " + str(state) + " Which is: " + goal_status_dict[state])
        if state == GoalStatus.ABORTED or state == GoalStatus.SUCCEEDED:
            done = True
    rospy.loginfo("Finished goal")  
    rospy.sleep(3)