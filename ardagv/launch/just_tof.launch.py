import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    pkg_3dtof_adtf31xx_dir = get_package_share_directory("adi_3dtof_adtf31xx")

    # Feature: added transformation to LaserScan within the adi_3dtof node
    adi_3dtof_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_3dtof_adtf31xx_dir, 'launch', 'adi_3dtof_adtf31xx_launch.py')
        ),
        launch_arguments={
            # LaserScan Arguments documentation: https://github.com/ros-perception/pointcloud_to_laserscan/tree/humble#parameters
            # [m] -> Minimal height scanning range
            "min_height,": "-0.05",
            # [m] -> Maximal height scanning range
            "max_height,": "0.05",
            # [rad] -> Minimal angle scanning range (-75 deg)
            "angle_min,": "-0.6544984694978736",
            # [rad] -> Maximal angle scanning range (75 deg)
            "angle_max,": "0.6544984694978736",
            # [rad] -> M_PI/180.0
            "angle_increment,": "0.0087",
            # [s] -> 30 Hz (1/30)
            "scan_time,": "0.0333333",
            # [m] -> Restrict min detection range
            "range_min,": "0.2",
            # [m] -> Restrict max detection range
            "range_max,": "5.0",
            #  If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.
            "use_inf,": "True",
            # Determines the value added to max range when use_infs parameter is set to false.
            "inf_epsilon,": "1.0",
        }.items(),
    )

    # Optional: Run the node with a namespace
    adi_3dtof_node_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('adi_3dtof_argdav'),
            adi_3dtof_node,
        ]
    )

    launch_description = LaunchDescription()

    launch_description.add_action(adi_3dtof_node)
    # launch_description.add_action(adi_3dtof_node_with_namespace)

    return launch_description
