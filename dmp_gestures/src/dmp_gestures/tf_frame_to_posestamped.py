#! /usr/bin/env python
"""
Created on 28/08/14

@author: Sam Pfeiffer

Given a frame name publish a posestamped
with it's pose from the TF transformation

"""
import sys
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
#from tf.transformations import 
import tf

if __name__ == '__main__':
    tf_frame = ""
    if len(sys.argv) < 2:
        print "Error, add a frame as argument, like:"
        print sys.argv[0] + " hand_right_grasping_frame"
        exit(0)
    else:
        tf_frame = str(sys.argv[1])
    rospy.init_node('frame_to_posestamped_')
    rospy.sleep(0.3) # Waiting for init node to do it's job
    transformer = tf.TransformListener()
    rospy.sleep(0.9)
    if not transformer.frameExists(tf_frame):
        print "Frame " + str(tf_frame) + " does not exist."
        exit(0)
    pub = rospy.Publisher('/tf_to_ps', PoseStamped)
    
    rospy.loginfo("Waiting for first available transform...")
    transformer.waitForTransform(tf_frame, 'base_link', rospy.Time(), rospy.Duration(20.0))
    rospy.loginfo("Starting!")
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        ps = PoseStamped()
        pos, quat = transformer.lookupTransform('base_link', tf_frame, rospy.Time())
        #rospy.loginfo("pos: " + str(pos) + " quat: " + str(quat))
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'base_link'
        ps.pose.position = Point(*pos)
        ps.pose.orientation = Quaternion(*quat)
        pub.publish(ps)
        r.sleep()
        