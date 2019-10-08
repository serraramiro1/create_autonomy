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
import rospy
from enum import Enum

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

ANGLE_TOLERANCE = 0.08

# factor that divides the effort to send the cmd_vel command
PID_ANGULAR_EFFORT_ADJUST = 10.0

DISTANCE_TOLERANCE = 0.1
PID_LINEAL_EFFORT_ADJUST = 10.0
PATH_THRESHOLD = 60

VEL_PUB_TOPIC = "/create1/cmd_vel"
GTS_SUB_TOPIC = "/create1/gts"

class STATES(Enum):
    FORWARD=1
    STOP_MOVING_FORWARD=2
    TURNING=3
    STOP_TURNING=4

def signal_handler(signal, frame, queue_size=1):
  # This will execute when Ctrl-c is pressed
    pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=1)
    aux = Twist()
    pub.publish(aux)
    sys.exit(0)


class CtrlNode:
    """A class that inits a node, in order to make the robot move to 4 fixed points, drawing a square
    """

    def __init__(self):

        rospy.init_node('control_node')

        #suscribers and publishers
        self._my_pid_angular_state_pub = rospy.Publisher(
            "/angular/state", Float64, queue_size=1)
        self._my_vel_pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=10)
        self._my_pid_lineal_state_pub = rospy.Publisher(
            "/lineal/state", Float64, queue_size=1)
        self._my_pid_angular_effort_sub = rospy.Subscriber(
            "/lineal/control_effort", Float64, self._lineal_effort_callback)
        self._my_pid_lineal_setpoint_pub = rospy.Publisher(
            "/lineal/setpoint", Float64, queue_size=10)
        self._my_gts_sub = rospy.Subscriber(
            GTS_SUB_TOPIC, Odometry, self._callback)
        self._my_pid_angular_effort_sub = rospy.Subscriber(
            "/angular/control_effort", Float64, self._angular_effort_callback)
        self._my_path_publisher = rospy.Publisher(
            "/create1/my_path", Path, queue_size=10)
        self._my_path = Path()
        self._my_pid_angular_setpoint_pub = rospy.Publisher(
            "/angular/setpoint", Float64, queue_size=1)

        self._path_threshold = PATH_THRESHOLD  # Throttle the path pub
        self._my_goals = [Pose2D()] * 4
        self._my_goals[1] = Pose2D(0, 2, 0)
        self._my_goals[2] = Pose2D(2, 2, 0)
        self._my_goals[3] = Pose2D(2, 0, 0)

        self._lineal_state = None
        self._goal_num = 0
        self._state = STATES.STOP_MOVING_FORWARD
        self._my_pose = Pose2D()
        self._my_pid_state = 0.0
        self._my_angle_goal = None
        self._my_pid_angular_effort = 0.0
        self._my_pid_lineal_effort = 0.0

    def _angular_effort_callback(self, data):
        """Callback for the angular effort calculated by the PID node

        Arguments:
            data {[Float64]} -- [Effort]
        """
        self._my_pid_angular_effort = data.data

    def _lineal_effort_callback(self, data):
        """Callback for the angular effort calculated by the PID node

        Arguments:
            data {[Float64]} -- [Effort]
        """
        self._my_pid_lineal_effort = data.data

    def _callback(self, data):
        """Callback for groundtruth

        Arguments:
            data {[Odometry]} -- [Odometry msg from gts]
        """
        self._my_pose.x = data.pose.pose.position.x  # Update pose
        self._my_pose.y = data.pose.pose.position.y
        q = data.pose.pose.orientation  # get orientation quaternion
        self._path_threshold -= 1
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        self._my_pose.theta = euler[2]  # theta=yaw
        if (self._path_threshold == 0):  # add the pose to path every PATH_THRESHOLD cycles
            aux = PoseStamped()
            aux.pose = data.pose.pose
            aux.header.frame_id = "/map"
            self._my_path.header.frame_id = "/map"
            self._my_path.poses.append(aux)
            self._my_path_publisher.publish(self._my_path)
            self._path_threshold = PATH_THRESHOLD

    def _move(self):
        if not (self._my_pose.x == None or self._my_pose.y == None):
            """State machine
            """
            self._my_pid_lineal_setpoint_pub.publish(0.0)
            self._my_pid_angular_setpoint_pub.publish(0.0)
            self._set_goal_angle()
            self._my_pid_lineal_state_pub.publish(self._diff_distance())
            self._my_pid_angular_state_pub.publish(self._diff_angle())

            if self._state == STATES.TURNING:
                self._turning()
                return
            if self._state == STATES.FORWARD:
                self._forward()
                return
            if self._state == STATES.STOP_MOVING_FORWARD:
                self._stop_moving_forward()
                return

    def _forward(self):
        """Makes the robot move forward
        """
        if self._reached_position():
            rospy.loginfo("Reached position")
            self._stop()
            self._state = STATES.STOP_MOVING_FORWARD
            return
        aux = Twist()
        aux.angular.z = -self._my_pid_angular_effort/PID_ANGULAR_EFFORT_ADJUST
        aux.linear.x = -self._my_pid_lineal_effort/PID_LINEAL_EFFORT_ADJUST
        self._my_vel_pub.publish(aux)

    def _stop(self):
        """Stops the robot
        """
        aux = Twist()
        self._my_vel_pub.publish(aux)

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

    def _stop_moving_forward(self):
        """Sets new goal
        """
        self._stop()
        self._goal_num += 1
        self._goal_num %= 4
        self._set_goal_angle()
        self._state = STATES.TURNING

    def _turning(self):
        """Turns while standing still
        """

        if (self._reached_angle()):
            rospy.loginfo("Reached angle!")
            self._stop()
            self._state = STATES.FORWARD

        else:
            aux = Twist()
            aux.angular.z = -self._my_pid_angular_effort/PID_ANGULAR_EFFORT_ADJUST
            self._my_vel_pub.publish(aux)

    def _set_goal_angle(self):
        """Sets angle goal basing on the actual position and goal position
        """
        self._my_angle_goal = (math.atan2(
            (self._my_goals[self._goal_num].y-self._my_pose.y), self._my_goals[self._goal_num].x-self._my_pose.x))

    def _reached_position(self):
        """Checks if robot is at the goal, with some tolerance

        Returns:
            [bool] -- [true=->robot is at the goal]
        """

        return (abs(self._diff_distance()) < DISTANCE_TOLERANCE)

    def _reached_angle(self):
        """Checks if the robot reached the required angle

        Returns:
            [bool] -- [True if reached required angle]
        """
        return (abs(self._diff_angle()) < ANGLE_TOLERANCE)

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
