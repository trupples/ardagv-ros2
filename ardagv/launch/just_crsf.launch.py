from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ardagv_crsf',
            executable='ardagv_crsf',
            ros_arguments=['-p', "uart:='/dev/ttymxc3'"],
            remappings=[
                ('/muxd_vel_unstamped', '/diff_drive_controller/cmd_vel_unstamped'),
                ('/muxd_fwd', '/forward_velocity_controller/command'),
                ('/cmd_vel_nav', '/cmd_vel'),
                ('/cmd_vel_nav_stamped', '/cmd_vel_stamped'),
            ]
        )
    ])

