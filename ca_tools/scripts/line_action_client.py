#! /usr/bin/env python

import rospy
import ca_tools.msg
import actionlib


def line_follower_client():

    client = actionlib.SimpleActionClient(
        'linefollower', ca_tools.msg.linefollowerAction)

    _duration_threshold = rospy.get_param("duration_threshold")
    # times oop stands for times out-of-path
    _times_oop = rospy.get_param("times_oop")
    # wait until the server is up
    client.wait_for_server()

    # create a sample goal
    goal = ca_tools.msg.linefollowerGoal(
        duration_threshold=_duration_threshold, times_oop=_times_oop)
    # send it to the server
    client.send_goal(goal)
    # wait until result is ready
    client.wait_for_result()
    # return the result
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('lineclient')

        result = line_follower_client()
        return_string = "Success! :)" if result.result else "Failure :'("
        rospy.loginfo("THE RESULT WAS : %s , AND THE TOTAL TIME WAS %d seconds",
                      return_string, result.total_time.to_sec())
    except rospy.ROSInterruptException:
        pass
