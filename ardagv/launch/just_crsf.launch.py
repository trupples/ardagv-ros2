from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = PathJoinSubstitution([get_package_share_directory("ardagv"), "config", "crsf.yaml"])
    return LaunchDescription([
        Node(
            package='ardagv_crsf',
            executable='ardagv_crsf',
            parameters=[params_file]
        )
    ])

