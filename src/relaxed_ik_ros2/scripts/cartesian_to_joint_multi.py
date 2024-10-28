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

class MultiCartesianConverter(Node):
	def __init__(self, input_array, output_file_full_path):
		super().__init__('cartesian_to_joint_multi')

		deault_setting_file_path = path_to_src + '/configs/settings.yaml'

		setting_file_path = deault_setting_file_path
		try:
			setting_file_path = self.get_parameter('setting_file_path')
		except:
			pass

		self.robot = Robot(setting_file_path)
		self.input_files = input_array
		self.current_index = 0
		self.output_path = output_file_full_path
		self.csv_reader = self.read_csv()

		self.relative_motion_end_x = 0.0
		self.relative_motion_end_y = 0.0
		self.relative_motion_end_z = 0.0

		self.motion_start_x = 0.2
		self.motion_start_y = 0.2
		self.motion_start_z = 0.7

		self.camera_x_offset = 0.0
		self.camera_y_offset = 0.0
		self.camera_z_offset = 0.0
		
		self.sub_sol = self.create_subscription(JointState, 'relaxed_ik/joint_angle_solutions', self.listener_event, 1)
		self.pub_goal = self.create_publisher(EEPoseGoals, '/relaxed_ik/ee_pose_goals', 1)
		self.pub_reset = self.create_publisher(JointState, '/relaxed_ik/reset', 1)


		self.output_data = []
		self.get_logger().info('Starting reading all pose data.')
		time.sleep(2)
		self.publish_new_goal()


	def create_output_file(self):
		with open(self.output_path, 'w+', newline='') as f:
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
			if (self.current_index == len(self.input_files)-1):
				self.get_logger().info('Finished reading all pose data. Writing to file')
				self.create_output_file()
			else:
				self.get_logger().info('Finished file. Moving to next file')
				self.next_csv()
			return

		ee_pose = EEPoseGoals()
		ee_pose.header.stamp = self.get_clock().now().to_msg()

		self.relative_motion_end_x = float(row[1])
		self.relative_motion_end_y = float(row[2])
		self.relative_motion_end_z = float(row[3])
		
		ee_pose.ee_poses = [Pose()]
		ee_pose.ee_poses[0].position.x = self.relative_motion_end_x + self.motion_start_x - self.camera_x_offset
		ee_pose.ee_poses[0].position.y = self.relative_motion_end_y + self.motion_start_y - self.camera_y_offset
		ee_pose.ee_poses[0].position.z = self.relative_motion_end_z + self.motion_start_z - self.camera_z_offset
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

	def next_csv(self):
		self.current_index += 1
		self.motion_start_x += self.relative_motion_end_x
		self.motion_start_y += self.relative_motion_end_y
		self.motion_start_z += self.relative_motion_end_z
		self.csv_reader = self.read_csv()
		time.sleep(5)
		self.publish_new_goal()

	def read_csv(self):
		file = open(self.input_files[self.current_index], 'r')
		reader = csv.reader(file)
		next(reader)
		return reader
	
if __name__ == '__main__':
	rclpy.init()

	# wait for relaxed IK to launch (bad coding practice)
	time.sleep(5)

	input_array = [os.path.join(os.path.expanduser("~"), "zoom_in_to_character", "scaled_smoothed_llm_output_trajectory_calmly_zoom in to character_1.txt.csv.csv.csv"),
				os.path.join(os.path.expanduser("~"), "pan_around_character", "llm_output_trajectory_playfully_pan around character_2.txt.csv")]
	out_path = os.path.join(os.path.expanduser("~"), "output", "calm_zoom_playful_pan.csv")

	cc = MultiCartesianConverter(input_array, out_path)

	rclpy.spin(cc)
	cc.destroy_node()
	rclpy.shutdown()
