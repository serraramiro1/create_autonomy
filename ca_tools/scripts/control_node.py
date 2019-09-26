#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math
from enum import Enum

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import tf

TURNING=1
STOP_TURNING=2
FORWARD=3
STOP_FORWARD=4

class ctrl_Node:
    
    def __init__(self):
        
        rospy.init_node('control_node')
        pubtopic = "/create1/cmd_vel"
        self.my_pub = rospy.Publisher(pubtopic, Twist, queue_size=10)
        subtopic = "/create1/gts"
        # Create a subscriber with appropriate topic, custom message and name of callback function.
        self.my_sub=rospy.Subscriber(subtopic,Odometry,self._callback)
        self.my_goals = []
        
        for i in range(4):
            self.my_goals.append(Pose2D())


        self.my_goals[0].x=0.0
        self.my_goals[0].y=0.0

        self.my_goals[1].x=2.0
        self.my_goals[1].y=0.0

        self.my_goals[2].x=2.0
        self.my_goals[2].y=2.0

        self.my_goals[3].x=0.0
        self.my_goals[3].y=2.0
        
        self.goal_num=0
        self.state=STOP_FORWARD
        self.my_pose=Pose2D()
        self.my_pose.x="unset"
        self.my_pose.y="unset"

# Create a callback function for the subscriber.
    def _callback(self,data):
        self.my_pose.x = data.pose.pose.position.x
        self.my_pose.y = data.pose.pose.position.y
        q0 = data.pose.pose.orientation.w
        q1 = data.pose.pose.orientation.x
        q2 = data.pose.pose.orientation.y
        q3 = data.pose.pose.orientation.z
        yaw = math.atan2(2.0*(q0*q3 + q1*q2),(1.0-2.0*(q2*q2 + q3*q3)))
        self.my_pose.theta = yaw
        self.move()
        #rospy.loginfo("My goal is %d, my state is %s",self.goal_num,self.state)

    def move(self):
        if self.state==TURNING:
            self.turning()
        elif self.state==STOP_TURNING:
            self.stop_turning()
        elif self.state==FORWARD:
            self.forward()
        elif self.state==STOP_FORWARD:
            self.stop_forward()

    
    def turning(self):

        if (self.reachedAngle()):
            self.stop()
            self.state=STOP_TURNING
            
        else:
            aux = Twist()
            if (self.my_angleGoal-self.my_pose.theta)<0:
                aux.angular.z=-1.0
            else:    
                aux.angular.z=1.0

            aux.angular.x=0.0
            aux.angular.y=0.0
            self.my_pub.publish(aux)

    def stop_turning(self):
        self.stop()
        self.moveForward()
        self.state=FORWARD





    def stop(self):
            aux = Twist()
            aux.angular.x=0.0
            aux.angular.y=0.0
            aux.angular.z=0.0
            aux.linear.x=0.0
            aux.linear.y=0.0
            aux.linear.z=0.0
            self.my_pub.publish(aux)
    
    def moveForward(self):
        aux = Twist()
        aux.angular.x=0.0
        aux.angular.y=0.0
        aux.angular.z=0.0
        aux.linear.x=0.2
        aux.linear.y=0.0
        aux.linear.z=0.0
        self.my_pub.publish(aux)

    def forward(self):
        if (self.reachedPosition()):
            self.stop()
            self.state=STOP_FORWARD
            self.goal_num+=1
            self.goal_num%=4
        else:
            self.setGoalAngle()
            if (not self.reachedAngle()):
                self.state=STOP_FORWARD
                self.stop()
            else:
                self.moveForward()
    
    def stop_forward(self):
         self.stop()
         self.setGoalAngle()
         self.state=TURNING


    def setGoalAngle(self):
        self.my_angleGoal=math.atan2((self.my_goals[self.goal_num].y-self.my_pose.y),self.my_goals[self.goal_num].x-self.my_pose.x)
        rospy.loginfo("Difference between angles is %f,",(self.my_angleGoal-self.my_pose.theta))
    
    def reachedAngle(self):
        return ((abs(self.my_angleGoal-self.my_pose.theta))<0.2)

    def reachedPosition(self):
        reached_y = ((abs(self.my_goals[self.goal_num].y-self.my_pose.y))<0.2)
        reached_x=(abs(self.my_goals[self.goal_num].x-self.my_pose.x)<0.2)
        return (reached_x and reached_y)


if __name__ == '__main__':
    node=ctrl_Node()
    rospy.spin()
