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

PID_EFFORT_ADJUST=5.0 ## factor that divides the effort to send the cmd_vel command
FORWARD=3
STOP_FORWARD=4
DISTANCE_TOLERANCE=0.2

pubtopic = "/create1/cmd_vel"
subtopic = "/create1/gts"

def signal_handler(signal, frame,queue_size=1):
  # This will execute when Ctrl-c is pressed
    pubtopic = "/create1/cmd_vel"
    pub = rospy.Publisher(pubtopic, Twist,queue_size=1)
    aux = Twist()
    pub.publish(aux)
    sys.exit(0)

class ctrl_Node:
    
    def __init__(self):
        
        rospy.init_node('control_node')
        self._my_pid_state_pub=rospy.Publisher("/state",Float64,queue_size=1)
        self._my_pub = rospy.Publisher(pubtopic, Twist, queue_size=10)

        # Create a subscriber with appropriate topic, custom message and name of callback function.
        self._my_sub=rospy.Subscriber(subtopic,Odometry,self._callback)
        self._my_pid_effort_sub=rospy.Subscriber("/control_effort",Float64,self._effortCallback)
        self._my_path_publisher= rospy.Publisher("/create1/my_path", Path, queue_size=10)
        self._my_path=Path()
        
        self._my_pid_setpoint_pub=rospy.Publisher("/setpoint",Float64,queue_size=1)
        self._my_pid_setpoint=Float64()
        self._my_pid_setpoint.data=0.0
        self._my_pid_setpoint_pub.publish(self._my_pid_setpoint)
        self._my_goals = []
        self._path_threshold=60
        for i in range(4):
            self._my_goals.append(Pose2D())

        self._my_pid_effort=0.0

        self._my_goals[1].x=2.0

        self._my_goals[2].x=2.0
        self._my_goals[2].y=2.0

        self._my_goals[3].y=2.0
        
        self._goal_num=0
        self._state=STOP_FORWARD
        self._my_pose=Pose2D()
        self._my_pose.x="unset"
        self._my_pose.y="unset"
        self._my_pid_state=0.0
        self._my_angleGoal=0.0
        self._my_pid_effort=0.0

    def _effortCallback(self,data):
        self._my_pid_effort=data.data



# Create a callback function for the subscriber.
    def _callback(self,data):
        self._my_pose.x = data.pose.pose.position.x
        self._my_pose.y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        self._path_threshold-=1
        if (self._path_threshold==0):
            aux = PoseStamped()
            aux.pose=data.pose.pose
            aux.header.frame_id="/map"
            self._my_path.header.frame_id="/map"
            self._my_path.poses.append(aux)
            self._my_path_publisher.publish(self._my_path)
            self._path_threshold=60
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        self._my_pose.theta=euler[2]
        self._my_pid_state=self._diffAngle()
        self._my_pid_setpoint.data=0.0
        self._my_pid_setpoint_pub.publish(self._my_pid_setpoint)
        self._my_pid_state_pub.publish(self._my_pid_state)
        self._move()

    def _move(self):

        if self._state==FORWARD:
            self._setGoalAngle()
            self._forward()
        elif self._state==STOP_FORWARD:
            self._stop_Forward()

    def _forward(self):
        aux=Twist()
        aux.angular.z=-self._my_pid_effort/PID_EFFORT_ADJUST
        aux.linear.x=0.2
        self._my_pub.publish(aux)
        if self._reachedPosition():
            self._state=STOP_FORWARD


    def _diffAngle(self):
        auxangle=(self._my_angleGoal-self._my_pose.theta)
        if auxangle< (-pi):
            auxangle+=(2*pi)
        elif auxangle > pi:
            auxangle-=(2*pi)
        return auxangle
    def _stop_Forward(self):
         self._goal_num+=1
         self._goal_num%=4
         self._setGoalAngle()
         self._state=FORWARD

    def _setGoalAngle(self):
        self._my_angleGoal=(math.atan2((self._my_goals[self._goal_num].y-self._my_pose.y),self._my_goals[self._goal_num].x-self._my_pose.x))


    def _reachedPosition(self):
        reached_y = ((abs(self._my_goals[self._goal_num].y-self._my_pose.y))<DISTANCE_TOLERANCE)
        reached_x=(abs(self._my_goals[self._goal_num].x-self._my_pose.x)<DISTANCE_TOLERANCE)
        return (reached_x and reached_y)

if __name__ == '__main__':
    node=ctrl_Node()
    signal.signal(signal.SIGINT, signal_handler) ##associates signal with handler
    rospy.spin()