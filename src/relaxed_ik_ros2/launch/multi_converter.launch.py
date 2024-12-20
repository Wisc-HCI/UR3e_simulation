from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch import LaunchDescription

path_to_src = get_package_share_directory('relaxed_ik_ros2') + '/relaxed_ik_core'
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
            package='relaxed_ik_ros2',
            namespace='',
            executable='relaxed_ik_rust.py',
            name='relaxed_ik_rust',
            parameters=[
                {'use_visualization': False,
                 'setting_file_path': setting_file_path}
            ]
        ),
        Node(
        	package='relaxed_ik_ros2',
        	executable='cartesian_to_joint_multi.py',
        	name='cartesian_to_joint_multi',
        	output='screen',
        	parameters=[{'setting_file_path': setting_file_path}]
        ),
    ])
