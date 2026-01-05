import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------
    # Robot description (xacro → urdf string)
    # ------------------------------------------------
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("robotic_arm"),
                "urdf",
                "robotic.xacro"
            ])
        ]),
        value_type=str
    )

    robot_description = {
        "robot_description": robot_description_content
    }

    # ------------------------------------------------
    # Gazebo Harmonic
    # ------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items()
    )

    # ------------------------------------------------
    # Clock bridge (ABSOLUTELY REQUIRED ✅)
    # ------------------------------------------------
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    # ------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ],
        output="screen"
    )

    # ------------------------------------------------
    # Spawn robot into Gazebo
    # ------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robotic_arm",
            "-topic", "robot_description",
            "-z", "0.0"
        ],
        output="screen"
    )

    # ------------------------------------------------
    # Controllers
    # ------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    # ------------------------------------------------
    # RViz
    # ------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # ------------------------------------------------
    # Launch order
    # ------------------------------------------------
    return LaunchDescription([
        gazebo,
        clock_bridge,
        rsp,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz
    ])
