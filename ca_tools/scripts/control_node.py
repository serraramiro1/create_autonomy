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


TURNING=1
STOP_TURNING=2
FORWARD=3
STOP_FORWARD=4

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
    
        self._my_pub = rospy.Publisher(pubtopic, Twist, queue_size=10)

        # Create a subscriber with appropriate topic, custom message and name of callback function.
        self._my_sub=rospy.Subscriber(subtopic,Odometry,self._callback)
        self._my_goals = []
        
        for i in range(4):
            self._my_goals.append(Pose2D())


        self._my_goals[0].x=0.0
        self._my_goals[0].y=0.0

        self._my_goals[1].x=2.0
        self._my_goals[1].y=0.0

        self._my_goals[2].x=2.0
        self._my_goals[2].y=2.0

        self._my_goals[3].x=0.0
        self._my_goals[3].y=2.0
        
        self._goal_num=0
        self._state=STOP_FORWARD
        self._my_pose=Pose2D()
        self._my_pose.x="unset"
        self._my_pose.y="unset"

# Create a callback function for the subscriber.
    def _callback(self,data):
        self._my_pose.x = data.pose.pose.position.x
        self._my_pose.y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        self._my_pose.theta=euler[2]
        self._move()
        #rospy.loginfo("My goal is %d, my _state is %s",self._goal_num,self._state)

    def _move(self):
        if self._state==TURNING:
            self._turning()
        elif self._state==STOP_TURNING:
            self._stop_Turning()
        elif self._state==FORWARD:
            self._forward()
        elif self._state==STOP_FORWARD:
            self._stop_Forward()

    
    def _turning(self):

        if (self._reachedAngle()):
            self._stop()
            self._state=STOP_TURNING
            
        else:
            aux = Twist()
            if (abs(self._my_angleGoal-self._my_pose.theta)>pi):
                negateFlag=-1.0
            else:
                negateFlag=1.0

            if (self._my_angleGoal-self._my_pose.theta)<0:
                aux.angular.z=negateFlag*-1.0
            else:    
                aux.angular.z=1.0

            rospy.loginfo("negateFlag is %f",negateFlag)
            self._my_pub.publish(aux)

    def _stop_Turning(self):
        self._stop()
        self._moveForward()
        self._state=FORWARD

    def _stop(self):
            aux = Twist()
            self._my_pub.publish(aux)
    
    def _moveForward(self):
        aux = Twist()
        aux.linear.x=0.2
        self._my_pub.publish(aux)

    def _forward(self):
        if (self._reachedPosition()):
            self._stop()
            self._state=STOP_FORWARD
            self._goal_num+=1
            self._goal_num%=4
        else:
            self._setGoalAngle()
            if (not self._reachedAngle()):
                self._state=STOP_FORWARD
                self._stop()
            else:
                self._moveForward()

    def _diffAngle(self):
        return (self._my_angleGoal-self._my_pose.theta)

    def _angleIsBig(self):
        return ((self._my_angleGoal-self._my_pose.theta)>1.0) and ((self._my_angleGoal-self._my_pose.theta)<((2*pi)-1.0))



    def _stop_Forward(self):
         self._stop()
         self._setGoalAngle()
         self._state=TURNING

    def _setGoalAngle(self):
        self._my_angleGoal=math.atan2((self._my_goals[self._goal_num].y-self._my_pose.y),self._my_goals[self._goal_num].x-self._my_pose.x)
        rospy.loginfo("Difference between angles is %f,",(self._my_angleGoal-self._my_pose.theta))
    
    def _reachedAngle(self):
        return ((abs(self._my_angleGoal-self._my_pose.theta))<0.2 or (abs(self._my_angleGoal-self._my_pose.theta))>(2*pi-0.2))

    def _reachedPosition(self):
        reached_y = ((abs(self._my_goals[self._goal_num].y-self._my_pose.y))<0.2)
        reached_x=(abs(self._my_goals[self._goal_num].x-self._my_pose.x)<0.2)
        return (reached_x and reached_y)


if __name__ == '__main__':
    node=ctrl_Node()
    signal.signal(signal.SIGINT, signal_handler) ##associates signal with handler
    rospy.spin()