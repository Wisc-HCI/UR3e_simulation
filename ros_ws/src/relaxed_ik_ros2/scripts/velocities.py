#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from relaxed_ik_ros2.msg import EEPoseGoals, EEVelGoals
import transformations as T
from robot import Robot
from ament_index_python.packages import get_package_share_directory

import pandas as pd
import os
from csv_to_dictionary import transform_csv

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'

class VelocityInput(Node):
	def __init__(self):
		super().__init__('velocity_input')

		deault_setting_file_path = path_to_src + '/configs/settings.yaml'

		setting_file_path = deault_setting_file_path
		try:
			setting_file_path = self.get_parameter('setting_file_path')
		except:
			pass

		self.robot = Robot(setting_file_path)

		self.ee_vel_goals_pub = self.create_publisher(EEVelGoals, 'relaxed_ik/ee_vel_goals', 5)

		# for now, plot out only the individual primitives and not their combined trajectory
		csv_path = "finetuned-data/baseline/5hz"
		csvf = "scene2/take4_calmly_tilt_low.csv"
		self.velocity_sets = transform_csv(csv_path, csvf)

		self.create_timer(0.1, self.publish_velocities)
		self.current_index = 0

	def publish_velocities(self):
		if self.current_index >= len(self.velocity_sets):
			msg = EEVelGoals()
			twist= Twist()

			twist.linear.x = 0.0
			twist.linear.y = 0.0
			twist.linear.z = 0.0
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = 0.0

			tolerance = Twist()
			msg.ee_vels.append(twist)
			msg.tolerances.append(tolerance)

			self.ee_vel_goals_pub.publish(msg)
			self.get_logger().info('Finished reading csv, stop moving the robot.')

			self.destroy_node()
			#rclpy.shutdown()
			return

		velocities = self.velocity_sets[self.current_index]
		self.get_logger().info('Index: {}'.format(self.current_index))
		msg = EEVelGoals()
		twist = Twist()

		twist.linear.x = float(velocities['linear'][0])
		twist.linear.y = float(velocities['linear'][1])
		twist.linear.z = float(velocities['linear'][2])
		twist.angular.x = float(velocities['angular'][0])
		twist.angular.y = float(velocities['angular'][1])
		twist.angular.z = float(velocities['angular'][2])

		tolerance = Twist()
		msg.ee_vels.append(twist)
		msg.tolerances.append(tolerance)
		self.ee_vel_goals_pub.publish(msg)
		self.get_logger().info('Publishing velocities: {}'.format(velocities))
		self.current_index = (self.current_index + 1) 

if __name__ == '__main__':
	rclpy.init()
	velocity_input = VelocityInput()
	rclpy.spin(velocity_input)
	velocity_input.destroy_node()
	rclpy.shutdown()
