from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    robot_description_filename = os.path.join(get_package_share_directory("ardagvmotor"), "urdf", "robot.urdf")
    robot_description_string = open(robot_description_filename).read()

    # canopen_core master
    """
    canopen_master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("ardagvmotor"),
                "config",
                "master.dcf",
            ),
            "bus_config": os.path.join(
                get_package_share_directory("ardagvmotor"),
                "config",
                "bus.yml",
            ),
            "can_interface_name": "can0",
        }.items(),
    )
    """

    # ros2_control controller_manager
    robot_control_config = os.path.join(get_package_share_directory("ardagvmotor"), "config", "ros2_controllers.yaml")
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_control_config,
            {"robot_description": robot_description_string}
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_string}
        ],
    )

    # For each of the items in ros2_controllers.yaml > controller_manager > ros__parameters, create a "spawner"
    spawners = []
    for to_spawn in [
        "joint_state_broadcaster",
        #"diff_drive_controller",
        #"cia402_controller_left",
        #"cia402_controller_right",
        "forward_velocity_controller"
        ]:
        spawner_node = Node(
                package = "controller_manager",
                executable = "spawner",
                arguments = [to_spawn, "--controller-manager", "/controller_manager"],
        )
        spawners.append(spawner_node)

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        *spawners
    ])

