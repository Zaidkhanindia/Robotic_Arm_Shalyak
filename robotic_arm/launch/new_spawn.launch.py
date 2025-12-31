import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------
    # Robot description (xacro → urdf)
    # ------------------------------------------------
    pkg_path = get_package_share_directory("robotic_arm")
    xacro_file = os.path.join(pkg_path, "urdf", "robotic.xacro")

    robot_description_content = xacro.process_file(xacro_file).toxml()

    robot_description = {
        "robot_description": robot_description_content
    }

    # ------------------------------------------------
    # Gazebo Harmonic
    # ------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
    )

    # ------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    # ------------------------------------------------
    # Spawn robot in Gazebo
    # ------------------------------------------------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robotic_arm",
            "-topic", "robot_description",
            "-z", "0.5"
        ],
        output="screen"
    )

    # ------------------------------------------------
    # ROS ↔ Gazebo bridge
    # ------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"
        ],
        output="screen"
    )

    # ------------------------------------------------
    # ROS2 Control Node
    # ------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("robotic_arm"),
                "config",
                "controllers.yaml"
            ])
        ],
        output="screen"
    )

    # ------------------------------------------------
    # Controller Spawners
    # ------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )

    # ------------------------------------------------
    # RViz
    # ------------------------------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    # ------------------------------------------------
    # Launch description
    # ------------------------------------------------
    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz
    ])
