import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        "ur_type",
        description="Type/series of used UR robot. Selects the committed plant description file.",
        choices=[
            "ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur10e", "ur12e",
            "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30",
        ],
        default_value="ur5e",
    ),
    DeclareLaunchArgument(
        "description_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_plant_description"), "models", LaunchConfiguration("ur_type"), "ur_plant.urdf"]
        ),
        description="Path to the plant's committed URDF description file.",
    ),
    DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_plant_bringup"), "config", "ur_plant_controllers.yaml"]
        ),
        description="Path to the ros2_control controllers configuration.",
    ),
    DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_plant_world"), "worlds", "ur_plant_world.sdf"]
        ),
        description="Absolute path to the Gazebo world file (pinned DART physics).",
    ),
    DeclareLaunchArgument(
        "activate_joint_controller",
        default_value="true",
        description="Activate the joint trajectory controller on startup.",
    ),
    DeclareLaunchArgument(
        "initial_joint_controller",
        default_value="scaled_joint_trajectory_controller",
        choices=["joint_trajectory_controller", "scaled_joint_trajectory_controller"],
        description="Initially loaded robot controller.",
    ),
    DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz.",
    ),
    DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
        ),
        description="RViz configuration file.",
    ),
    DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Start Gazebo with GUI. False starts a headless server.",
    ),
]


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    world_file = LaunchConfiguration("world_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # The committed URDF is read from the file directly — no xacro, no code
    # generation at launch time.
    robot_description_content = Command(["cat ", description_file])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # The broadcaster publishes the clean joint states; remapped to /raw_joint_states
    # (ground truth). The Stage 2 plant_sensors node republishes the noisy plant data.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        remappings=[("/joint_states", "/raw_joint_states")],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("activate_joint_controller")),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(LaunchConfiguration("activate_joint_controller")),
    )

    # Fixed spawn name "ur" — the bridge YAML's gz topic names depend on it.
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-file", description_file, "-name", "ur"],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": IfElseSubstitution(
            LaunchConfiguration("gazebo_gui"),
            [" -r -v 4 ", world_file],
            [" -s -r -v 4 ", world_file],
        )}.items(),
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("ur_plant_bringup"), "config", "bridges.yaml"]
                )
            }
        ],
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        initial_joint_controller_spawner_stopped,
        gz_spawn_entity,
        gz_sim,
        ros_gz_bridge_node,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
