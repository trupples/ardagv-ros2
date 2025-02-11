from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('adi_3dtof_adtf31xx'), 'launch', 'adi_3dtof_adtf31xx_launch.py')),
            launch_arguments = {
                # 'a': 'b' ???
            }.items()
        )
    ])

