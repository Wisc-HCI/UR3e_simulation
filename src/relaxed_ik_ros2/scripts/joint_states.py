#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from robot import Robot
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
import csv
import os
import time

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'

class JointStatePublisher(Node):
	def __init__(self, csv_path, csvf):
		super().__init__('joint_state_publisher')

		deault_setting_file_path = path_to_src + '/configs/settings.yaml'

		setting_file_path = deault_setting_file_path
		try:
			setting_file_path = self.get_parameter('setting_file_path')
		except:
			pass

		self.robot = Robot(setting_file_path)
		self.csv_path = csv_path
		self.csvf = csvf
		self.csv_reader = self.read_csv()
		
		self.publisher_ = self.create_publisher(JointState, 'joint_states', 5)
		self.timer = self.create_timer(0.1, self.timer_callback)
		self.get_logger().info(f'Reading joint states from {self.csvf}')

	def read_csv(self):
		script_dir = os.path.dirname(os.path.abspath(__file__))
		ws_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))) # ros2_ws
		pkg_path = os.path.join(ws_path, 'src/relaxed_ik_ros2')
		full_csv_path = os.path.join(pkg_path, csv_path, csvf)
		self.get_logger().info(f'{full_csv_path}')
		file = open(full_csv_path, 'r')
		reader = csv.reader(file)
		next(reader)
		return reader
		
	def timer_callback(self):
		try:
			row = next(self.csv_reader)
		except StopIteration:
			self.get_logger().info('Finished reading all joint states.')
			self.timer.cancel()
			return
		
		joint_state = JointState()
		joint_state.header.stamp = self.get_clock().now().to_msg()
		joint_state.name = self.robot.all_joint_names
		joint_state.position = [float(val) for val in row[1:1+len(self.robot.all_joint_names)]]
		#self.get_logger().info(f'length: {len(joint_state.position)}')
		joint_state.velocity = []
		joint_state.effort = []
		self.publisher_.publish(joint_state)
		
		
	
if __name__ == '__main__':
	rclpy.init()
	csv_path = "example-joint-state-data/"
	csvf = "example.csv"
	joint_state_publisher = JointStatePublisher(csv_path, csvf)
	rclpy.spin(joint_state_publisher)
	joint_state_publisher.destroy_node()
	rclpy.shutdown()
