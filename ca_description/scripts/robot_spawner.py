#!/usr/bin/env python

import os
import re
import rospy
from random import randint
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler

class RobotSpawner(object):

  SPAWN_URDF_TOPIC = '/gazebo/spawn_urdf_model'
  TIMEOUT_S = rospy.Duration(secs=10.0)

  def __init__(self):
    rospy.init_node('robot_spawner')
    ns = rospy.get_namespace()

    # Get robot index
    try:
      index = re.findall('[0-9]+', ns)[0]
    except IndexError:
      index = 0
    i = index

    # Spawn URDF service client
    rospy.wait_for_service(RobotSpawner.SPAWN_URDF_TOPIC)
    try:
      spawn_urdf_model = rospy.ServiceProxy(RobotSpawner.SPAWN_URDF_TOPIC, SpawnModel)

      # Filling model spawner request
      msg = SpawnModelRequest()

      # Model name
      msg.model_name = "irobot_create2.{}".format(i)

      # Robot information from robot_description
      robot_description_param = "/create{}/robot_description".format(i)

      # Wait for params
      pose_param = "{}/amcl".format(ns)
      initial_time = rospy.Time.now()
      while RobotSpawner.TIMEOUT_S > (rospy.Time.now() - initial_time):
        if rospy.has_param(robot_description_param) and rospy.has_param(pose_param):
          break

      msg.model_xml = rospy.get_param(robot_description_param)

      msg.robot_namespace = ns

      # Using pose from parameter server
      msg.initial_pose = Pose()
      msg.initial_pose.position.x = rospy.get_param(
            "{}/initial_pose_x".format(pose_param), randint(-10, 10))
      msg.initial_pose.position.y = rospy.get_param(
            "{}/initial_pose_y".format(pose_param), randint(-10, 10))
      msg.initial_pose.position.z = rospy.get_param(
            "{}/initial_pose_z".format(pose_param), 0.)
      yaw = rospy.get_param(
            "{}/initial_pose_a".format(pose_param), 0.)
      q = quaternion_from_euler(0, 0, yaw)
      msg.initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

      msg.reference_frame = "world"

      # Spawn model
      res = spawn_urdf_model(msg)
      print(res.status_message)
    except rospy.ServiceException:
      print("Could not spawn {}".format(msg.model_name))
      exit(1)

    print("{} spawned correctly".format(msg.model_name))

def main():
  try:
    RobotSpawner()
  except rospy.ROSInterruptException: pass

if __name__ == "__main__":
  main()