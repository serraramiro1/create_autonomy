#! /usr/bin/env python

import rospy

import actionlib
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist, Pose2D
from nav_msgs.msg import Odometry
import ca_tools.msg


class LineFollowerActionServer(object):
    """A class that implements an action server, in order to
    make the robot follow two lines
    """
    IS_RED_TOPIC = "/isred"
    _feedback = ca_tools.msg.linefollowerFeedback()
    _result = ca_tools.msg.linefollowerResult()
    LINEAR_VEL = 0.12  # linear vel
    ANGULAR_VEL = 0.2
    IS_BARRIER_TOPIC = "/isbarrier"
    VEL_PUB_TOPIC = "/create1/cmd_vel"
    RIGHT_SENSOR_SUB_TOPIC = "/color_sensor_plugin/right_color_sensor"
    LEFT_SENSOR_SUB_TOPIC = "/color_sensor_plugin/left_color_sensor"
    GTS_SUBTOPIC = "/create1/gts"  # Groundtruth topic
    # Tolerance accepted between the robot pose and the goal
    distance_tolerance = 0.4

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ca_tools.msg.linefollowerAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._my_vel_pub = rospy.Publisher(
            self.VEL_PUB_TOPIC, Twist, queue_size=10)
        self._my_sub = rospy.Subscriber(
            self.GTS_SUBTOPIC, Odometry, self._gts_callback)
        self._last_pose = Pose()
        self._current_pose = Pose()
        self._accumulated_distance = 0.0
        self._duration_threshold = 0.0
        self._times_oop = 0
        self._times_oop_threshold = 0
        self._starting_time = rospy.Time.now()
        # suscribers and publishers
        self._my_vel_pub = rospy.Publisher(
            self.VEL_PUB_TOPIC, Twist, queue_size=10)

        self._my_right_sensor_sub = rospy.Subscriber(
            self.RIGHT_SENSOR_SUB_TOPIC, Bool, self._right_sensor_callback)
        self._is_red_sub = rospy.Subscriber(
            self.IS_RED_TOPIC, Bool, self._is_red_callback)

        self._is_barrier_sub = rospy.Subscriber(
            self.IS_BARRIER_TOPIC, Bool, self._is_barrier_callback)
        self._my_left_sensor_sub = rospy.Subscriber(
            self.LEFT_SENSOR_SUB_TOPIC, Bool, self._left_sensor_callback)

        self._has_reached_goal = False
        self._is_barrier = False
        self._is_left_detected = False
        self._is_right_detected = False
        self._move_flag = False
        self._goal = Pose2D(-1.75, 0.18, 0)

    def _gts_callback(self, data):
        self._last_pose = self._current_pose
        self._current_pose = data.pose.pose
        self._accumulated_distance += math.hypot(self._last_pose.position.x - self._current_pose.position.x,
                                                 self._last_pose.position.y - self._current_pose.position.y)

    def _is_red_callback(self, data):
        self._has_reached_goal = data.data
    
    def _is_barrier_callback(self, data):   
        self._is_barrier = data.data

    def _left_sensor_callback(self, data):
        self._is_left_detected = data.data

    def _right_sensor_callback(self, data):
        self._is_right_detected = data.data

    def _move_forward(self):
        aux = Twist()
        aux.linear.x = self.LINEAR_VEL
        self._my_vel_pub.publish(aux)
        return

    def _move_left(self):
        aux = Twist()
        aux.angular.z = self.ANGULAR_VEL
        aux.linear.x = self.LINEAR_VEL / 20
        self._my_vel_pub.publish(aux)

    def _move_right(self):
        aux = Twist()
        aux.angular.z = -self.ANGULAR_VEL
        aux.linear.x = self.LINEAR_VEL / 20
        self._my_vel_pub.publish(aux)

    def move(self):
        """Public method that makes the robot move,
        it won't do nothing until a goal is recieved
        """
        if self._is_barrier:
            self._stop_moving()
            return
        if(self._move_flag):
            if (self._is_left_detected and self._is_right_detected):
                self._move_forward()
                return
            if(self._is_left_detected):
                self._move_right()
                return
            if(self._is_right_detected):
                self._move_left()
                return
            self._move_forward()
            return

    def _stop_moving(self):
        """Stops the robot
        """
        for x in range(0, 8):  # Just to make sure it stops
            aux = Twist()
            aux.linear.x = 0
            self._my_vel_pub.publish(aux)

    def execute_cb(self, goal):
        """Function to be called when a goal is recieved

        Arguments:
            goal {[ca_tools.msg.linefollowerGoal]} -- [Goal message]
        """
        # helper variables
        r = rospy.Rate(10)
        success = True
        self._accumulated_distance = 0.0
        self._move_flag = True
        self._starting_time = rospy.Time.now()

        self._duration_threshold = rospy.Duration(goal.duration_threshold)
        self._times_oop_threshold = goal.times_oop
        self._times_oop = 0
        self._feedback.time_elapsed = rospy.Duration()
        # publish info to the console for the user
        rospy.loginfo("Starting the line-following race")

        # start executing the action
        while (not self._has_reached_goal):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            if (self._feedback.time_elapsed > self._duration_threshold):
                self._result.result = False
                self._result.total_time = rospy.Time.now() - self._starting_time
                self._as.set_preempted(self._result)
                rospy.loginfo(
                    '%s: Preemted, because the mission lasted more than expected' % self._action_name)
                success = False
                break

            self._feedback.time_elapsed = rospy.Time.now()-self._starting_time
            self._feedback.distance_moved = self._accumulated_distance
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()  # repeate every 1 sec

        if success:

            self._result.total_time = rospy.Time.now() - self._starting_time
            rospy.loginfo('%s: Succeeded', self._action_name)
            self._result.result = True
            self._as.set_succeeded(self._result)

        self._stop_moving()
        self._move_flag = False


if __name__ == '__main__':

    rospy.init_node('linefollower')
    server = LineFollowerActionServer(rospy.get_name())
    rate = rospy.Rate(200)
    while(not rospy.is_shutdown()):
        server.move()
        rate.sleep()
