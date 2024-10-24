#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
from relaxed_ik_ros2.msg import EEPoseGoals
import csv
import os
import time

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'

class CartesianConverter(Node):
	def __init__(self):
		super().__init__('cartesian_to_joint')


		self.initial_x = 0.2
		self.initial_y = 0.2
		self.initial_z = 0.7

		self.x_offset = 0.0
		self.y_offset = 0.0
		self.z_offset = 0.0
		
		self.sub_sol = self.create_subscription(JointState, 'relaxed_ik/joint_angle_solutions', self.listener_event, 1)
		self.pub_goal = self.create_publisher(EEPoseGoals, '/relaxed_ik/ee_pose_goals', 1)

		self.publish_new_goal()




	def listener_event(self, msg):
		self.get_logger().info("RECEIVE: {}".format(msg))


	def publish_new_goal(self):

		ee_pose = EEPoseGoals()
		
		ee_pose.ee_poses = [Pose()]
		ee_pose.ee_poses[0].position.x = 1.0
		ee_pose.ee_poses[0].position.y = 2.0
		ee_pose.ee_poses[0].position.z = 3.0
		ee_pose.ee_poses[0].orientation.w = 0.707
		ee_pose.ee_poses[0].orientation.x = 0.0
		ee_pose.ee_poses[0].orientation.y = 0.707
		ee_pose.ee_poses[0].orientation.z = 0.0

		self.pub_goal.publish(ee_pose)

	
if __name__ == '__main__':
	rclpy.init()

	cc = CartesianConverter()
	rclpy.spin(cc)
	cc.destroy_node()
	rclpy.shutdown()
