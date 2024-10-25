#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
from relaxed_ik_ros2.msg import EEPoseGoals
from robot import Robot
import csv
import os
import time

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'

class CartesianConverter(Node):
	def __init__(self, csv_path, csvf, output_path):
		super().__init__('cartesian_to_joint')

		deault_setting_file_path = path_to_src + '/configs/settings.yaml'

		setting_file_path = deault_setting_file_path
		try:
			setting_file_path = self.get_parameter('setting_file_path')
		except:
			pass
		
		self.robot = Robot(setting_file_path)
		self.csv_path = csv_path
		self.csvf = csvf
		self.output_path = output_path
		
		self.initial_x = 0.2
		self.initial_y = 0.2
		self.initial_z = 0.7

		self.x_offset = 0.0
		self.y_offset = 0.0
		self.z_offset = 0.0
		
		self.sub_sol = self.create_subscription(JointState, 'relaxed_ik/joint_angle_solutions', self.listener_event, 1)
		self.pub_goal = self.create_publisher(EEPoseGoals, '/relaxed_ik/ee_pose_goals', 1)

		# csv_input should be a list of lists
		self.csv_input = self.read_csv()
		
		#self.get_logger().info("csv_input: {}".format(self.csv_input))
		
		# this is where publish_new_goal() is going to save its outputs
		self.output = []

		time.sleep(2)
		self.publish_all_goals()
		
		#self.save_to_csv()


	def read_csv(self):
		full_csv_path = os.path.join(self.csv_path, self.csvf)
		self.get_logger().info("Reading from: {}".format(full_csv_path))
		
		with open(full_csv_path, 'r') as f:
			 reader = csv.reader(f)
			 next(reader)
			 data = [row for row in reader]
		return data
			 

	def listener_event(self, msg):
		self.output.append(msg.position)
		self.get_logger().info("RECEIVE: {}".format(msg))


	def publish_all_goals(self):
		for row in self.csv_input:
			self.publish_new_goal(row)
			self.get_logger().info("Waiting for listener callback to receive position data") 
			time.sleep(0.5)
			rclpy.spin_once(self)
		
		self.get_logger().info("All received positions: {}".format(self.output))
		
		# should call write_to_csv() here
		self.save_to_csv()
		
	def publish_new_goal(self, row):

		self.get_logger().info("Which row, {}".format(row))
		
		ee_pose = EEPoseGoals()
		
		ee_pose.header.stamp = self.get_clock().now().to_msg()
		
		ee_pose.ee_poses = [Pose()]
		ee_pose.ee_poses[0].position.x = float(row[1]) + self.initial_x - self.x_offset
		ee_pose.ee_poses[0].position.y = float(row[2]) + self.initial_y - self.y_offset
		ee_pose.ee_poses[0].position.z = float(row[3]) + self.initial_z - self.z_offset
		ee_pose.ee_poses[0].orientation.w = 0.707
		ee_pose.ee_poses[0].orientation.x = 0.0
		ee_pose.ee_poses[0].orientation.y = 0.707
		ee_pose.ee_poses[0].orientation.z = 0.0

		self.pub_goal.publish(ee_pose)
		#self.get_logger().info("ee_pose, {}".format(ee_pose))
		self.get_logger().info("Ready to publish")
	
	
	def save_to_csv(self):
		full_csv_path = os.path.join(self.output_path, csvf)
		
		with open(full_csv_path, mode = "w", newline = "") as f:
			writer = csv.writer(f)
			for inner_list in self.output:
				writer.writerow(inner_list)
		'''
		for inner_list in self.output:
			self.get_logger().info("inner_list: {}".format(inner_list))
		'''
		self.get_logger().info("DONE")
	
		
	
if __name__ == '__main__':
	rclpy.init()

	csv_path = os.path.join(os.path.expanduser("~"), "zoom_in_to_character")
	csvf = "scaled_smoothed_llm_output_trajectory_calmly_zoom in to character_1.txt.csv.csv.csv"
	output_path = os.path.join(os.path.expanduser("~"), "output")
	
	cc = CartesianConverter(csv_path, csvf, output_path)
	rclpy.spin(cc)
	cc.destroy_node()
	rclpy.shutdown()
