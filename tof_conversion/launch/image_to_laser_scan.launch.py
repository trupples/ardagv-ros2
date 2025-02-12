import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tof_conversion_dir = get_package_share_directory("tof_conversion")

    # pkg_nav2_dir = get_package_share_directory("nav2_bringup")
    # pkg_tb3_sim = get_package_share_directory("tb3_sim")

    # use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    # autostart = LaunchConfiguration("autostart", default="True")

    # nav2_launch_cd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_tof_conversion_dir, "launch", "image_to_laser_scan.launch.py")
    #     ),
    #     launch_arguments={
    #         "use_sim_time": use_sim_time,
    #         "autostart": autostart,
    #         "map": os.path.join(pkg_tb3_sim, "maps", "map_simulation.yaml"),
    #     }.items()
    # )

    # Pre-defined rviz with the navigation2 configuration
    depth_image_to_laserscan_cmd = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan_node",
        arguments=[
            "-d" + os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
    )

    set_init_amcl_pose_cmd = Node(
        package="tb3_sim",
        executable="amcl_init_pose_publisher",
        name="amcl_init_pose_publisher",
        parameters=[{
            "x": -2.0,
            "y": -0.5,
        }]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(nav2_launch_cd)
    launch_description.add_action(rviz_launch_cmd)
    launch_description.add_action(set_init_amcl_pose_cmd)

    return launch_description
