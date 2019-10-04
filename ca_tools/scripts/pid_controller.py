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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

ANGLE_TOLERANCE = 0.1
ANGULAR_VEL = 1.0
PID_ANGULAR_EFFORT_ADJUST = 5.0  # factor that divides the effort to send the cmd_vel command
TURNING = 2
FORWARD = 3
STOP_FORWARD = 4
DISTANCE_TOLERANCE = 0.2
PID_LINEAL_EFFORT_ADJUST = 10.0

pubtopic = "/create1/cmd_vel"
subtopic = "/create1/gts"


def signal_handler(signal, frame, queue_size=1):
  # This will execute when Ctrl-c is pressed
    pubtopic = "/create1/cmd_vel"
    pub = rospy.Publisher(pubtopic, Twist, queue_size=1)
    aux = Twist()
    pub.publish(aux)
    sys.exit(0)


class Ctrl_Node:
    """A class that inits a node, in order to make the robot move to 4 fixed points, drawing a square
    """

    def __init__(self):

        rospy.init_node('control_node')

        #suscribers and publishers
        self._my_pid_angular_state_pub = rospy.Publisher(
            "/angular/state", Float64, queue_size=1)
        self._my_pub = rospy.Publisher(pubtopic, Twist, queue_size=10)
        self._my_pid_lineal_state_pub = rospy.Publisher(
            "/lineal/state", Float64, queue_size=1)
        self._my_pid_angular_effort_sub = rospy.Subscriber(
            "/lineal/control_effort", Float64, self._lineal_effort_callback)
        self._my_pid_lineal_setpoint_pub=rospy.Publisher("/lineal/setpoint",Float64,queue_size=10)
        self._my_sub = rospy.Subscriber(subtopic, Odometry, self._callback)
        self._my_pid_angular_effort_sub = rospy.Subscriber(
            "/angular/control_effort", Float64, self._angular_effort_callback)
        self._my_path_publisher = rospy.Publisher(
            "/create1/my_path", Path, queue_size=10)
        self._my_path = Path()
        self._my_pid_angular_setpoint_pub = rospy.Publisher("/angular/setpoint", Float64, queue_size=1)

        self._my_goals = []
        self._path_threshold = 60 #Throttle the path pub
        #CREATE GOALS
        for i in range(4):
            self._my_goals.append(Pose2D())
        self._my_goals[1].x = 2.0
        self._my_goals[2].x = 2.0
        self._my_goals[2].y = 2.0
        self._my_goals[3].y = 2.0

        self._lineal_state=Float64()
        self._goal_num = 0
        self._state = STOP_FORWARD
        self._my_pose = Pose2D()
        self._my_pose.x = "unset"
        self._my_pose.y = "unset"
        self._my_pid_state = 0.0
        self._my_angle_goal = 0.0
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
        if (self._path_threshold == 0):  # add the pose to path every 60 cycles
            aux = PoseStamped()
            aux.pose = data.pose.pose
            aux.header.frame_id = "/map"
            self._my_path.header.frame_id = "/map"
            self._my_path.poses.append(aux)
            self._my_path_publisher.publish(self._my_path)
            self._path_threshold = 60

        self._my_pid_lineal_state_pub.publish(self._diff_distance())
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        self._my_pose.theta = euler[2]  # theta=yaw
        self._my_pid_lineal_setpoint_pub.publish(0.0)
        self._my_pid_angular_setpoint_pub.publish(0.0)
        self._my_pid_angular_state_pub.publish(self._diff_angle())
        self._move()  # Calls state machine

    def _move(self):
        """State machine
        """
        if self._state == FORWARD:
            self._set_goal_angle()
            self._forward()
        elif self._state == STOP_FORWARD:
            self._stop_Forward()
        elif self._state == TURNING:
            self._turning()
        

    def _forward(self):
        """Makes the robot move forward
        """
        aux = Twist()
        aux.angular.z = -self._my_pid_angular_effort/PID_ANGULAR_EFFORT_ADJUST
        aux.linear.x = -self._my_pid_lineal_effort/PID_LINEAL_EFFORT_ADJUST
        self._my_pub.publish(aux)
        if self._reached_position():
            self._state = STOP_FORWARD
            self._stop()

    def _stop(self):
        """Stops the robot
        """
        aux = Twist()
        self._my_pub.publish(aux)

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

    def _stop_Forward(self):
        """Sets new goal
        """
        self._goal_num += 1
        self._goal_num %= 4
        self._set_goal_angle()
        self._state = TURNING

    def _turning(self):
        """Turns while standing still
        """

        if (self._reached_angle()):
            self._stop()
            self._state = FORWARD

        else:
            aux = Twist()
            aux.angular.z = -self._my_pid_angular_effort/PID_ANGULAR_EFFORT_ADJUST
            self._my_pub.publish(aux)

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
        reached_y = (
            (abs(self._my_goals[self._goal_num].y-self._my_pose.y)) < DISTANCE_TOLERANCE)
        reached_x = (
            abs(self._my_goals[self._goal_num].x-self._my_pose.x) < DISTANCE_TOLERANCE)
        return (reached_x and reached_y)

    def _reached_angle(self):
        """Checks if the robot reached the required angle

        Returns:
            [bool] -- [True if reached required angle]
        """
        return (abs(self._diff_angle()) < ANGLE_TOLERANCE)

    def _diff_distance(self):
        """Returns the distance beteween the robot and the goal
        """
        return(math.hypot(self._my_goals[self._goal_num].x-self._my_pose.x,self._my_goals[self._goal_num].y-self._my_pose.y))


if __name__ == '__main__':
    node = Ctrl_Node()
    # associates signal with handler
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()
