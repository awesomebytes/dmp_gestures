#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thursday Aug 8 10:10:55 2013

@author: sampfeiffer
"""
#import sys
#import actionlib
import rospy
import rosbag
from datetime import datetime
from razer_hydra.msg import Hydra
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav_msgs.msg import Path

HYDRA_DATA_TOPIC = '/hydra_calib'
HAND_GRASP_CONTROLLER_RIGHT_AS = '/right_hand_controller/grasp_posture_controller'
HAND_GRASP_CONTROLLER_LEFT_AS = '/left_hand_controller/grasp_posture_controller'

RIGHT_HAND_POSESTAMPED_TOPIC = '/teleop_right_hand_pose'
LEFT_HAND_POSESTAMPED_TOPIC = '/teleop_left_hand_pose'
RIGHT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_right_hand_pose_reference'
LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_left_hand_pose_reference'

PATH3D_TOPIC = '/path3d'
POSEARRAY_3D_TOPIC = '/posearray3d'

RIGHT_HAND_INITIAL_POINT = Point(x=0.6, y=-0.2, z=1.1)
LEFT_HAND_INITIAL_POINT = Point(x=0.6, y=0.2, z=1.1)


# rosrun tf static_transform_publisher 0.0 -1.0 1.0 0 0 0 /base_link /hydra_base 100

class RazerControl():

    def __init__(self):
        self.pub_right_hand_pose = rospy.Publisher(RIGHT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_right_hand_pose_reference = rospy.Publisher(RIGHT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose = rospy.Publisher(LEFT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose_reference = rospy.Publisher(LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.hydra_data_subs = rospy.Subscriber(HYDRA_DATA_TOPIC, Hydra, self.hydraDataCallback)
        self.pub_path3d = rospy.Publisher(PATH3D_TOPIC, Path, latch=True)
        self.pub_posearray3d = rospy.Publisher(POSEARRAY_3D_TOPIC, PoseArray, latch=True)
#        self.hand_grasp_client = actionlib.SimpleActionClient(HAND_GRASP_CONTROLLER_AS, GraspHandPostureExecutionAction)
#        rospy.loginfo("Waiting for " + HAND_GRASP_CONTROLLER_AS)
#        self.hand_grasp_client.wait_for_server()
#        rospy.loginfo("Connected to " + HAND_GRASP_CONTROLLER_AS)
        self.last_hydra_message = None
        self.writing_bag = False
        self.bag_name = "unitialized" 
        self.bag = None
        self.path3d = Path()
        self.posearray3d = PoseArray()


    def hydraDataCallback(self, data):
        #rospy.loginfo("Received data from " + HYDRA_DATA_TOPIC)
        self.last_hydra_message = data
        tmp_pose_right = PoseStamped()
        tmp_pose_right.header.frame_id = 'base_link'
        tmp_pose_right.header.stamp = rospy.Time.now()
        tmp_pose_right.pose.position.x = self.last_hydra_message.paddles[1].transform.translation.x
        tmp_pose_right.pose.position.y = self.last_hydra_message.paddles[1].transform.translation.y
        tmp_pose_right.pose.position.z = self.last_hydra_message.paddles[1].transform.translation.z
        tmp_pose_right.pose.position.x += RIGHT_HAND_INITIAL_POINT.x
        tmp_pose_right.pose.position.y += RIGHT_HAND_INITIAL_POINT.y
        tmp_pose_right.pose.position.z += RIGHT_HAND_INITIAL_POINT.z
        tmp_pose_right.pose.orientation = self.last_hydra_message.paddles[1].transform.rotation
        
        tmp_pose_left = PoseStamped()
        tmp_pose_left.header.frame_id = 'base_link'
        tmp_pose_left.header.stamp = rospy.Time.now()
        tmp_pose_left.pose.position.x = self.last_hydra_message.paddles[0].transform.translation.x
        tmp_pose_left.pose.position.y = self.last_hydra_message.paddles[0].transform.translation.y
        tmp_pose_left.pose.position.z = self.last_hydra_message.paddles[0].transform.translation.z
        tmp_pose_left.pose.position.x += LEFT_HAND_INITIAL_POINT.x
        tmp_pose_left.pose.position.y += LEFT_HAND_INITIAL_POINT.y
        tmp_pose_left.pose.position.z += LEFT_HAND_INITIAL_POINT.z
        
        tmp_pose_left.pose.orientation = self.last_hydra_message.paddles[0].transform.rotation
        if self.last_hydra_message.paddles[1].buttons[0] == True:
            self.pub_right_hand_pose.publish(tmp_pose_right)
        if self.last_hydra_message.paddles[0].buttons[0] == True:
            self.pub_left_hand_pose.publish(tmp_pose_left)
            
        self.pub_right_hand_pose_reference.publish(tmp_pose_right)
        self.pub_left_hand_pose_reference.publish(tmp_pose_left)
        if self.writing_bag:
#            self.bag.write(LEFT_HAND_POSESTAMPED_TOPIC, tmp_pose_left)
            self.bag.write(RIGHT_HAND_POSESTAMPED_TOPIC, tmp_pose_right)
            self.path3d.poses.append(tmp_pose_right)
            self.posearray3d.poses.append(Pose(tmp_pose_right.pose.position, tmp_pose_right.pose.orientation))

    def run(self):
        rospy.loginfo("Press LB / RB to send the current pose")
        
        while self.last_hydra_message == None:
            rospy.sleep(0.1)
            
        rospy.loginfo("Got the first data of the razer... Now we can do stuff")
            
        sleep_rate=0.1 # check at 10Hz
        left_pushed = right_pushed = False
        
        while True:
#            if self.last_hydra_message.paddles[1].buttons[1] == True and self.current_thumb_status != 'up':
            # TODO: When pressing button rb/lb, paddles[0/1].buttons[0], record posestampeds in a list like a gesture
            # write those posestampeds to a file
#            if self.last_hydra_message.paddles[0].buttons[1] == True and not left_pushed: # left paddle
#                left_pushed = True
#                self.bag_name = datetime.now().isoformat()
#                self.bag = rosbag.Bag(bag_name, 'w')
#                self.writing_bag = True
#                
#            elif self.last_hydra_message.paddles[0].buttons[1] == False and left_pushed:
#                left_pushed = False
#                self.writing_bag = False
#                self.bag.close()
#                
            if self.last_hydra_message.paddles[1].buttons[0] == True and not right_pushed:
                right_pushed = True
                self.bag_name = datetime.now().isoformat() + ".bag"
                self.bag = rosbag.Bag(self.bag_name, 'w')
                self.writing_bag = True
                self.path3d.header.stamp = rospy.Time.now()
                self.path3d.header.frame_id = 'base_link'
                self.posearray3d.header = self.path3d.header
                rospy.loginfo("Recording bag: " + self.bag_name)
                
            elif self.last_hydra_message.paddles[1].buttons[0] == False and right_pushed:
                right_pushed = False
                self.writing_bag = False
                self.bag.write(PATH3D_TOPIC, self.path3d)
                self.bag.write(POSEARRAY_3D_TOPIC, self.posearray3d)
                self.bag.close()
                rospy.loginfo("Closing bag: " + self.bag_name)
                self.pub_path3d.publish(self.path3d)
                rospy.loginfo("Published path3d in topic: " + PATH3D_TOPIC)
                self.pub_posearray3d.publish(self.posearray3d)
                rospy.loginfo("Published posearray in topic: " + POSEARRAY_3D_TOPIC)
                
                
            rospy.sleep(sleep_rate)


if __name__ == '__main__':
    rospy.init_node('hydra_info')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    node = RazerControl()
    node.run()

#     rospy.spin()