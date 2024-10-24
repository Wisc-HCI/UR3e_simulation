#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from robot import Robot
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
	def __init__(self, csv_path, csvf, out_path):
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
		self.output_path = out_path
		self.csv_reader = self.read_csv()


		self.initial_x = 0.2
		self.initial_y = 0.2
		self.initial_z = 0.7

		self.x_offset = 0.0
		self.y_offset = 0.0
		self.z_offset = 0.0
		
		self.sub_sol = self.create_subscription(JointState, 'relaxed_ik/joint_angle_solutions', self.listener_event, 1)
		self.pub_goal = self.create_publisher(EEPoseGoals, '/relaxed_ik/ee_pose_goals', 1)
		self.pub_reset = self.create_publisher(JointState, '/relaxed_ik/reset', 1)

		self.get_logger().info('Starting reading all pose data.')
		self.publish_new_goal()

		self.output_data = []

	def create_output_file(self):
		# script_dir = os.path.dirname(os.path.abspath(__file__))
		# ws_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))) # ros2_ws
		# pkg_path = os.path.join(ws_path, 'src/UR3e_simulation/src')
		# full_csv_path = os.path.join(pkg_path, self.output_path, csvf)
		full_csv_path = os.path.join(self.output_path, csvf)
		self.get_logger().info(f'{full_csv_path}')
		with open(full_csv_path, 'w+', newline='') as f:
			writer = csv.writer(f, delimiter=',')
			for line in self.output_data:
				writer.writerow(line)
		self.get_logger().info('Done')

	def reset(self):

		return

	def listener_event(self, msg):
		# msg.name
		# msg.position
		# msg.velocity
		self.output_data.append(msg.position)
		self.get_logger().info("RECEIVE: {}".format(msg))
		self.publish_new_goal()

	def publish_new_goal(self):
		try:
			row = next(self.csv_reader)
		except StopIteration:
			self.get_logger().info('Finished reading all pose data. Writing to file')
			self.create_output_file()
			return
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

		ee_pose.tolerances = [Twist()]
		ee_pose.tolerances[0].linear.x = 0.0
		ee_pose.tolerances[0].linear.y = 0.0
		ee_pose.tolerances[0].linear.z = 0.0
		ee_pose.tolerances[0].angular.x = 0.0
		ee_pose.tolerances[0].angular.y = 0.0
		ee_pose.tolerances[0].angular.z = 0.0

		# print(ee_pose)
		self.get_logger().info("SEND: {}".format(ee_pose))
		self.pub_goal.publish(ee_pose)

	def read_csv(self):
		# script_dir = os.path.dirname(os.path.abspath(__file__))
		# ws_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))) # ros2_ws
		# pkg_path = os.path.join(ws_path, 'src/UR3e_simulation/src')
		# full_csv_path = os.path.join(pkg_path, csv_path, csvf)
		full_csv_path = os.path.join(csv_path, csvf)
		self.get_logger().info(f'{full_csv_path}')
		file = open(full_csv_path, 'r')
		reader = csv.reader(file)
		next(reader)
		return reader
	
if __name__ == '__main__':
	rclpy.init()

	# wait for relaxed IK to launch (bad coding practice)
	time.sleep(5)

	# more bad coding
	# csv_path = os.path.join(os.path.expanduser("~"), "pan_around_character")
	csv_path = os.path.join(os.path.expanduser("~"), "zoom_in_to_character")
	csv_path = '/workspace'
	out_path = os.path.join(os.path.expanduser("~"), "output")
	out_path = '/workspace'
	# csvf = "scaled_smoothed_llm_output_trajectory_calmly_pan around character_7.txt.csv.csv.csv"
	csvf = "scaled_smoothed_llm_output_trajectory_calmly_zoom in to character_1.txt.csv.csv.csv"
	cc = CartesianConverter(csv_path, csvf, out_path)
	rclpy.spin(cc)
	cc.destroy_node()
	rclpy.shutdown()
