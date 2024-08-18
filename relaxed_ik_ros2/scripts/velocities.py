#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from relaxed_ik_ros2.msg import EEPoseGoals, EEVelGoals
import transformations as T
from robot import Robot
from ament_index_python.packages import get_package_share_directory

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

        #print("! ", setting_file_path)
        self.robot = Robot(setting_file_path)


        self.ee_vel_goals_pub = self.create_publisher(EEVelGoals, 'relaxed_ik/ee_vel_goals', 5)

        self.velocity_sets = [
        {'linear': [1, 0.0, 0.0], 'angular':[0.0, 0.0, 0.1]},
        {'linear': [2, 0.0, 0.0], 'angular':[0.0, 0.0, 0.2]},
        {'linear': [0.0, 1, 0.0], 'angular':[0.0, 0.1, 0.0]},
        {'linear': [0.0, 0.0, 1], 'angular':[0.1, 0.0, 0.0]},
        {'linear': [1, 1, 0.0], 'angular':[0.0, 0.0, 0.1]},
        ]

        #self.seq = 1

        #self.linear = [0.0,0.0,0.0]
        #self.angular = [0.0,0.0,0.0]

        self.create_timer(1.0, self.publish_velocities)
        self.current_index = 0
        
    def publish_velocities(self):
        velocities = self.velocity_sets[self.current_index]
        msg = EEVelGoals()
        twist = Twist()
        twist.linear.x = velocities['linear'][0]
        twist.linear.y = velocities['linear'][1]
        twist.linear.z = velocities['linear'][2]
        twist.angular.x = velocities['angular'][0]
        twist.angular.y = velocities['angular'][1]
        twist.angular.z = velocities['angular'][2]

        tolerance = Twist()
        msg.ee_vels.append(twist)
        msg.tolerances.append(tolerance)

        self.ee_vel_goals_pub.publish(msg)
        self.get_logger().info('Publishing velocities: {}'.format(velocities))

        self.current_index = (self.current_index + 1) % len(self.velocity_sets)
    	
if __name__ == '__main__':
    rclpy.init()
    velocity_input = VelocityInput()
    rclpy.spin(velocity_input)
    velocity_input.destroy_node()
    rclpy.shutdown()
