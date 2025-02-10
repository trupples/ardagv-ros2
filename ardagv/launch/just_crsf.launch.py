from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ardagv_crsf',
            executable='ardagv_crsf',
            ros_arguments=['-p', "uart:='/dev/ttymxc3'"],
        )
    ])

