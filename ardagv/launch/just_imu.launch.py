from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_ros2',
            executable='imu_ros2_node',
            ros_arguments=['-p', "iio_context_string:='local:'"]
        )
    ])

