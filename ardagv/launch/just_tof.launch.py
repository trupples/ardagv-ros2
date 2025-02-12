import os

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_3dtof_adtf31xx_dir = get_package_share_directory("adi_3dtof_adtf31xx")

    depthimage_to_laserscan_cfg = os.path.join(
        get_package_share_directory('ardagv'), 'cfg', 'depthimage_to_laserscan.yaml'
    )

    adi_3dtof_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_3dtof_adtf31xx_dir, 'launch', 'adi_3dtof_adtf31xx_launch.py')
        ),
        launch_arguments={
            # 'a': 'b' ???
        }.items()
    )

    depthimage_to_laserscan_cmd = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan_node",
        parameters=[depthimage_to_laserscan_cfg],
        remappings=[
            ('depth', '/cam1/depth_image'),
            ('depth_camera_info', '/cam1/camera_info'),
        ]
    )

    launch_description = LaunchDescription()

    launch_description.add_action(adi_3dtof_cmd)
    launch_description.add_action(depthimage_to_laserscan_cmd)

    return launch_description
