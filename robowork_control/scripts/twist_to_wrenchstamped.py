#!/usr/bin/python

import rospy
import numpy

import geometry_msgs
from geometry_msgs.msg import *

twist_stamped_publisher = None
twist_frame_id = None
wrench_force_scaler = 1.0
wrench_moment_scaler = 1.0

def twist_in_cb(twist_msg):
  global twist_stamped_publisher
  global twist_frame_id
  global wrench_force_scaler
  global wrench_moment_scaler

  wrench_stamped_msg = geometry_msgs.msg.WrenchStamped()
  wrench_stamped_msg.header.frame_id = twist_frame_id
  wrench_stamped_msg.header.stamp = rospy.Time.now()
  wrench_stamped_msg.wrench.force.x = - twist_msg.linear.x * wrench_force_scaler
  wrench_stamped_msg.wrench.force.y = - twist_msg.linear.y * wrench_force_scaler
  wrench_stamped_msg.wrench.force.z = - twist_msg.linear.z * wrench_force_scaler
  wrench_stamped_msg.wrench.torque.x = - twist_msg.angular.x * wrench_moment_scaler
  wrench_stamped_msg.wrench.torque.y = - twist_msg.angular.y * wrench_moment_scaler
  wrench_stamped_msg.wrench.torque.z = - twist_msg.angular.z * wrench_moment_scaler

  twist_stamped_publisher.publish(wrench_stamped_msg)

def main():
  global twist_stamped_publisher
  global twist_frame_id
  global wrench_force_scaler
  global wrench_moment_scaler

  rospy.init_node('twist_to_wrenchstamped')

  twist_frame_id = rospy.get_param('~twist_frame_id')
  wrench_force_scaler = rospy.get_param('~wrench_force_scaler')
  wrench_moment_scaler = rospy.get_param('~wrench_moment_scaler')

  rospy.Subscriber("twist_in", geometry_msgs.msg.Twist, twist_in_cb)
  twist_stamped_publisher = rospy.Publisher("wrenchstamped_out", geometry_msgs.msg.WrenchStamped, queue_size=1)

  rospy.spin()

if __name__ == '__main__': main()
