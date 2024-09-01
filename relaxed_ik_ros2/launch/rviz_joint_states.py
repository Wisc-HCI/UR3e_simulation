from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch import LaunchDescription

path_to_relaxed_ik = get_package_share_directory('relaxed_ik_ros2')
setting_file_path = path_to_relaxed_ik + '/relaxed_ik_core/configs/example_settings/ur3e.yaml'

def generate_launch_description():
    # Load the infomation
    setting_file = open(setting_file_path, 'r')
    settings = yaml.load(setting_file, Loader=yaml.FullLoader)

    urdf_path = path_to_relaxed_ik + '/relaxed_ik_core/configs/urdfs/' + settings["urdf"]
    urdf_file = open(urdf_path, 'r')
    urdf_string = urdf_file.read()

    print("demo.launch: using setting file path", str(setting_file_path))

    return LaunchDescription([ 
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', path_to_relaxed_ik + "/rviz/relaxed_ik_viewer.rviz"],
        ),
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path],
            parameters=[{'publish_frequency': 50.0, 
                        'robot_description': urdf_string}]
        ),
        Node(
            package='relaxed_ik_ros2',
            namespace='',
            executable='rviz_viewer.py',
            name='rviz_viewer',
            output='screen',
            parameters=[{'setting_file_path': setting_file_path}]
        ),
        Node(
        	package='relaxed_ik_ros2',
        	executable='joint_states.py',
        	name='joint_states',
        	output='screen',
        	parameters=[{'setting_file_path': setting_file_path}]
        ),
  
    ])
