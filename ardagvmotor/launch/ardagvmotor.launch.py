def generate_launch_description():
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
                "ardagvmotor",
                "master.dcf",
            ),
            "bus_config": os.path.join(
                get_package_share_directory("ardagvmotor"),
                "config",
                "ardagvmotor",
                "bus.yml",
            ),
            "can_interface_name": "can1",
        }.items(),
    )

    robot_description = {"robot_description": None}
    robot_control_config = os.path.join(get_package_share_directory("ardagvmotor"), "config", "ardagvmotor", "ros2_controllers.yaml")
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription([
        canopen_master,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
        robot_state_publisher,
    ])

