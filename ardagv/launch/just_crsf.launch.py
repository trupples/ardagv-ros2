from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ardagv_crsf',
            executable='ardagv_crsf',
            ros_arguments=['-p', "uart:='/dev/ttymxc3'"],
            remappings=[
                ('/muxd_vel', '/diff_drive_controller/cmd_vel'),
                ('/muxd_fwd', '/forward_velocity_controller/command')
            ]
        )
    ])

