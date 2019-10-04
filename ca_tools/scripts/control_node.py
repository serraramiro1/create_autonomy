#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Ramiro Serra

import math
import signal
import sys
from math import pi

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


TURNING = 1
STOP_TURNING = 2
FORWARD = 3
STOP_FORWARD = 4
LINEAR_VEL = 0.2
DISTANCE_TOLERANCE = 0.2
ANGLE_TOLERANCE = 0.2
ANGULAR_VEL = 1.0
pubtopic = "/create1/cmd_vel"
subtopic = "/create1/gts"
BIG_ANGLE_THRESHOLD = 1.0  # minimum angle in which the robot will stop and turn


def signal_handler(signal, frame, queue_size=1):
  # This will execute when Ctrl-c is pressed
    pub = rospy.Publisher(pubtopic, Twist, queue_size=1)
    aux = Twist()
    pub.publish(aux)
    sys.exit(0)


class Ctrl_Node:

    def __init__(self):

        rospy.init_node('control_node')

        self._my_pub = rospy.Publisher(pubtopic, Twist, queue_size=10)

        # Create a subscriber with appropriate topic, custom message and name of callback function.
        self._my_sub = rospy.Subscriber(subtopic, Odometry, self._callback)
        self._my_goals = []

        for i in range(4):
            self._my_goals.append(Pose2D())

        self._my_goals[1].x = 2.0

        self._my_goals[2].x = 2.0
        self._my_goals[2].y = 2.0

        self._my_goals[3].y = 2.0

        self._goal_num = 0
        self._state = STOP_FORWARD
        self._my_pose = Pose2D()
        self._my_pose.x = "unset"
        self._my_pose.y = "unset"

# Create a callback function for the subscriber.
    def _callback(self, data):
        self._my_pose.x = data.pose.pose.position.x
        self._my_pose.y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        self._my_pose.theta = euler[2]
        self._move()

    def _move(self):
        if self._state == TURNING:
            self._turning()
        elif self._state == STOP_TURNING:
            self._stop_turning()
        elif self._state == FORWARD:
            self._forward()
        elif self._state == STOP_FORWARD:
            self._stop_forward()

    def _turning(self):

        if (self._reached_angle()):
            self._stop()
            self._state = STOP_TURNING

        else:
            aux = Twist()
            if (abs(self._my_anglegoal-self._my_pose.theta) > pi):
                negateflag = -1.0
            else:
                negateflag = 1.0

            if (self._my_anglegoal-self._my_pose.theta) < 0:
                aux.angular.z = negateflag*-ANGULAR_VEL
            else:
                aux.angular.z = ANGULAR_VEL

            rospy.loginfo("negateflag is %f", negateflag)
            self._my_pub.publish(aux)

    def _stop_turning(self):
        self._stop()
        self._move_forward()
        self._state = FORWARD

    def _stop(self):
        aux = Twist()
        self._my_pub.publish(aux)

    def _move_forward(self):
        aux = Twist()
        aux.linear.x = LINEAR_VEL
        self._my_pub.publish(aux)

    def _forward(self):
        if (self._reached_position()):
            self._stop()
            self._state = STOP_FORWARD
            self._goal_num += 1
            self._goal_num %= 4
        else:
            self._set_goal_angle()
            if (not self._reached_angle()):
                self._state = STOP_FORWARD
                self._stop()
            else:
                self._move_forward()

    def _diff_angle(self):
        return (self._my_anglegoal-self._my_pose.theta)

    def _angle_is_big(self):
        return ((self._my_anglegoal-self._my_pose.theta) > BIG_ANGLE_THRESHOLD) and ((self._my_anglegoal-self._my_pose.theta) < ((2*pi)-BIG_ANGLE_THRESHOLD))

    def _stop_forward(self):
        self._stop()
        self._set_goal_angle()
        self._state = TURNING

    def _set_goal_angle(self):
        self._my_anglegoal = math.atan2(
            (self._my_goals[self._goal_num].y-self._my_pose.y), self._my_goals[self._goal_num].x-self._my_pose.x)
        rospy.loginfo("Difference between angles is %f,",
                      (self._my_anglegoal-self._my_pose.theta))

    def _reached_angle(self):
        return ((abs(self._my_anglegoal-self._my_pose.theta)) < ANGLE_TOLERANCE or (abs(self._my_anglegoal-self._my_pose.theta)) > (2*pi-ANGLE_TOLERANCE))

    def _reached_position(self):
        reached_y = (
            (abs(self._my_goals[self._goal_num].y-self._my_pose.y)) < DISTANCE_TOLERANCE)
        reached_x = (
            abs(self._my_goals[self._goal_num].x-self._my_pose.x) < DISTANCE_TOLERANCE)
        return (reached_x and reached_y)


if __name__ == '__main__':
    node = Ctrl_Node()
    # associates signal with handler
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()
