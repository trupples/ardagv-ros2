from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    # Declare launch arguments
    decl_name = DeclareLaunchArgument('name', default_value='ardagv')
    decl_can_iface = DeclareLaunchArgument('can_iface', default_value='can0')
    decl_config_dir = DeclareLaunchArgument('config_dir', default_value=os.path.join(get_package_share_directory("ardagv_motors"), "config"))
    decl_controller = DeclareLaunchArgument('controller', default_value='diff_drive_controller')

    # Get launch arguments
    name = LaunchConfiguration('name')
    can_iface = LaunchConfiguration('can_iface')
    config_dir = LaunchConfiguration('config_dir')
    controller = LaunchConfiguration('controller')

    # Compute URDF from macros
    robot_description = Command([
        'xacro ', PathJoinSubstitution([get_package_share_directory("ardagv"), "urdf", "ardagv_all.urdf.xacro"]),
        ' name:=', name,
        ' can_iface:=', can_iface,
        ' config_dir:=', config_dir
    ])

    # Launch all the things!

    # ros2_control controller_manager
    robot_control_config = PathJoinSubstitution([get_package_share_directory("ardagv"), "config", "ros2_controllers.yaml"])
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_control_config,
            #{"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
        output="both",
        remappings=[
            ("controller_manager:__node", [name, TextSubstitution(text="_controller_manager")]), # Rename from /controller_manager to /ardagv_controller_manager
            ("~/robot_description", "/robot_description"),
        ],
        # use_global_arguments = False # Important, otherwise all children of controller_manager will also get the same __node:= remap and we will have nonsensical duplicate nodes
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager", executable = "spawner",
        arguments = ["joint_state_broadcaster", "--controller-manager", [TextSubstitution(text="/"), name, TextSubstitution(text="_controller_manager")]],
        output="both",
    )

    # diff_drive_controller or forward_velocity_controller
    controller_spawner = Node(
        package = "controller_manager", executable = "spawner",
        arguments = [LaunchConfiguration("controller"), "--controller-manager", [TextSubstitution(text="/"), name, TextSubstitution(text="_controller_manager")]],
        output="both",
    )

    return LaunchDescription([
        decl_name,
        decl_can_iface,
        decl_config_dir,
        decl_controller,
        controller_manager_node,
        robot_state_publisher,
        controller_spawner,
        joint_state_broadcaster_spawner,
    ])

