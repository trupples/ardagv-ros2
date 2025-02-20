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
    decl_can_iface = DeclareLaunchArgument('can_iface', default_value='can1')
    decl_controller = DeclareLaunchArgument('controller', default_value='diff_drive_controller')

    # Get launch arguments
    name = LaunchConfiguration('name')
    can_iface = LaunchConfiguration('can_iface')

    # Compute URDF from macros
    robot_description = Command([
        'xacro ', PathJoinSubstitution([get_package_share_directory("ardagv"), "urdf", "ardagv_all.urdf.xacro"]),
        ' name:=', name,
        ' can_iface:=', can_iface,
        ' sim_mode:=', 'False',
    ])

    # Launch all the things!

    # Twist mux
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        remappings={('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')},
        parameters=[PathJoinSubstitution([get_package_share_directory("ardagv"), "config", "twist_mux.yaml"])]
    )

    # ros2_control controller_manager
    robot_control_config = PathJoinSubstitution([get_package_share_directory("ardagv"), "config", "ros2_controllers.yaml"])
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_control_config
        ],
        remappings=[('~/robot_description', '/robot_description')],
        output="both"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str), 'use_sim_time': False}
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager", executable = "spawner",
        arguments = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # diff_drive_controller or forward_velocity_controller
    controller_spawner = Node(
        package = "controller_manager", executable = "spawner",
        arguments = [LaunchConfiguration("controller"), "--controller-manager", "/controller_manager"],
        output="both",
    )
    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(get_package_share_directory("ardagv"), 'config/ekf.yaml'), {'use_sim_time': False}]
    )

    return LaunchDescription([
        decl_name,
        decl_can_iface,
        decl_controller,
        controller_manager_node,
        robot_state_publisher,
        controller_spawner,
        joint_state_broadcaster_spawner,
        # robot_localization_node,
        twist_mux_node
    ])

