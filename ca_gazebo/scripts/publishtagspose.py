#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

#Publishes the tag 1 pose relative to the /map frame, to "tags_gts/pose" topic
if __name__ == '__main__':
    rospy.init_node('tags_tf_to_pose')

    listener = tf.TransformListener()

    tag_pose = rospy.Publisher('tags_gts/pose', geometry_msgs.msg.PoseStamped,queue_size=1)

    rate = rospy.Rate(10.0)
    pose = geometry_msgs.msg.PoseStamped()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/tag_1', rospy.Time(0))
            pose.pose.position.x=trans[0]
            pose.pose.position.y=trans[1]
            pose.pose.position.z=trans[2]
            pose.header.frame_id="/map"
            pose.pose.orientation.x=rot[0]
            pose.pose.orientation.y=rot[1]
            pose.pose.orientation.z=rot[2]
            pose.pose.orientation.w=rot[3]

            tag_pose.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()