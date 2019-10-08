#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) Ekumen
# Released under the BSD License.
#
# Authors:
#   * Ramiro Serra

import math
import signal
import sys
from math import pi
from enum import Enum

import rospy
from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion




LINEAR_VEL = 0.2
DISTANCE_TOLERANCE = 0.2
ANGLE_TOLERANCE = 0.2
ANGULAR_VEL = 1.0
VEL_PUB_TOPIC = "/create1/cmd_vel"
GTS_SUBTOPIC = "/create1/gts"
BIG_ANGLE_THRESHOLD = 1.0  # minimum angle in which the robot will stop and turn


class STATES(Enum):
    FORWARD=1
    STOP_FORWARD=2
    TURNING=3
    STOP_TURNING=4


def signal_handler(signal, frame, queue_size=1):
  # This will execute when Ctrl-c is pressed
    pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=1)
    aux = Twist()
    pub.publish(aux)
    sys.exit(0)


class CtrlNode:

    def __init__(self):

        rospy.init_node('control_node')
        
        self._my_pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=10)
        # Create a subscriber with appropriate topic, custom message and name of callback function.
        self._my_sub = rospy.Subscriber(
            GTS_SUBTOPIC, Odometry, self._gts_callback)
        self._my_goals = [Pose2D()] * 4
        self._my_goals[1] = Pose2D(0, 2, 0)
        self._my_goals[2] = Pose2D(2, 2, 0)
        self._my_goals[3] = Pose2D(2, 0, 0)

        self._goal_num = 0
        self.states = STATES.STOP_FORWARD
        self._my_pose = Pose2D()
        self._my_pose.x = None
        self._my_pose.y = None
        self._my_angle_goal = 0.0

    def _gts_callback(self, data):
        self._my_pose.x = data.pose.pose.position.x
        self._my_pose.y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        self._my_pose.theta = euler[2]

    def _move(self):
        if not (self._my_pose.x == None or self._my_pose.y == None):
            if self.states == STATES.TURNING:
                self._turning()
                return
            if self.states == STATES.STOP_TURNING:
                self._stop_turning()
                return
            if self.states == STATES.FORWARD:
                self._forward()
                return
            if self.states == STATES.STOP_FORWARD:
                self._stop_forward()
                return

    def _turning(self):

        if (self._reached_angle()):
            self._stop()
            self.states = STATES.STOP_TURNING

        else:
            aux = Twist()
            if (abs(self._my_angle_goal-self._my_pose.theta) > pi):
                negateflag = -1.0
            else:
                negateflag = 1.0

            if (self._my_angle_goal-self._my_pose.theta) < 0:
                aux.angular.z = negateflag*-ANGULAR_VEL
            else:
                aux.angular.z = ANGULAR_VEL

            self._my_pub.publish(aux)

    def _stop_turning(self):
        self._stop()
        self._move_forward()
        self.states = STATES.FORWARD

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
            self.states = STATES.STOP_FORWARD
            self._goal_num += 1
            self._goal_num %= 4
        else:
            self._set_goal_angle()
            if (not self._reached_angle()):
                self.states = STATES.STOP_FORWARD
                self._stop()
            else:
                self._move_forward()

    def _diff_angle(self):
        """Calculates the difference between the angle goal and current angle

        Returns:
            [float] -- [Difference of angles in radians, wrapped between -pi/2 and pi/2]
        """
        auxangle = (self._my_angle_goal-self._my_pose.theta)

        while (abs(auxangle) > pi):
            if auxangle < (-pi):
                auxangle += (2*pi)
            elif auxangle > pi:
                auxangle -= (2*pi)
        return auxangle

    def _angle_is_big(self):
        return (abs(self._diff_angle()) > BIG_ANGLE_THRESHOLD)

    def _stop_forward(self):
        self._stop()
        self._set_goal_angle()
        self.states = STATES.TURNING

    def _set_goal_angle(self):
        self._my_angle_goal = math.atan2(
            (self._my_goals[self._goal_num].y-self._my_pose.y), self._my_goals[self._goal_num].x-self._my_pose.x)

    def _reached_angle(self):
        return (abs(self._diff_angle()) < ANGLE_TOLERANCE)

    def _reached_position(self):
        reached_y = (
            (abs(self._my_goals[self._goal_num].y-self._my_pose.y)) < DISTANCE_TOLERANCE)
        reached_x = (
            abs(self._my_goals[self._goal_num].x-self._my_pose.x) < DISTANCE_TOLERANCE)
        return (reached_x and reached_y)

    def _reached_position(self):
        """Checks if robot is at the goal, with some tolerance
        Returns:
        [bool] -- [true=->robot is at the goal]
        """

        return (abs(self._diff_distance()) < DISTANCE_TOLERANCE)

    def _diff_distance(self):
        """Returns the distance beteween the robot and the goal
        """
        return(math.hypot(self._my_goals[self._goal_num].x-self._my_pose.x, self._my_goals[self._goal_num].y-self._my_pose.y))


if __name__ == '__main__':
    node = CtrlNode()
    # associates signal with handler
    signal.signal(signal.SIGINT, signal_handler)
    while not rospy.is_shutdown():
        rospy.sleep(0.3)
        node._move()
