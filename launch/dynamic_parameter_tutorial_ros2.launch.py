import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def get_file_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)

def load_file(package_name, file_path):
    absolute_file_path = get_file_path(package_name, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def generate_launch_description():
    yaml_file = get_file_path('dynamic_parameter_tutorials', 'config/dynamic_tutorials_node.yaml')

    # Node
    # dynamic_tutorials_nodeノード
    dynamic_tutorials_node = Node(
        package='dynamic_parameter_tutorials',
        executable='dynamic_tutorials_node',
        name='dynamic_tutorials_node',
        parameters=[yaml_file],
        output='screen',
    )
    
    # rqt_reconfigure node
    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen'
    )

    return LaunchDescription([
        dynamic_tutorials_node,
        rqt_reconfigure_node
    ])