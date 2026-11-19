import math
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def generate_launch_description():
    package_dir = get_package_share_directory("practice03_odometry")
    turtlebot_package_dir = get_package_share_directory(
        "webots_ros2_turtlebot"
    )

    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    duration = LaunchConfiguration("duration")
    output_dir = LaunchConfiguration("output_dir")
    experiment_name = LaunchConfiguration("experiment_name")

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]),
        mode=mode,
        ros2_supervisor=False,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": '<robot name=""><link name=""/></robot>',
            "use_sim_time": True,
        }],
    )

    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "base_footprint",
        ],
    )

    map_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "1", "--y", "0", "--z", "0",
            "--yaw", str(math.pi / 2),
            "--pitch", "0", "--roll", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    supervisor_driver = WebotsController(
        robot_name="ground_truth_supervisor",
        parameters=[{
            "robot_description": os.path.join(
                package_dir, "resource", "ground_truth_supervisor.urdf"
            ),
            "use_sim_time": True,
        }],
        respawn=True,
    )

    turtlebot_driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {
                "robot_description": os.path.join(
                    turtlebot_package_dir,
                    "resource",
                    "turtlebot_webots.urdf",
                ),
                "use_sim_time": True,
                "set_robot_state_publisher": True,
            },
            os.path.join(
                turtlebot_package_dir, "resource", "ros2control.yml"
            ),
        ],
        remappings=[
            ("/diffdrive_controller/cmd_vel", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ],
        respawn=True,
    )

    diffdrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diffdrive_controller",
            "--controller-ros-args=-p "
            "diffdrive_controller:wheel_separation_multiplier:=1.1",
            "--controller-manager-timeout", "50",
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "50",
        ],
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=[diffdrive_spawner, joint_state_spawner],
    )

    velocity_publisher = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--use-sim-time",
            "--rate", "10",
            "/cmd_vel", "geometry_msgs/msg/TwistStamped",
            "{header: {stamp: now, frame_id: base_link}, "
            "twist: {linear: {x: 0.2}, angular: {z: 0.2}}}",
        ],
        output="log",
    )

    recorder = Node(
        package="practice03_odometry",
        executable="odometry_recorder",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "duration": ParameterValue(duration, value_type=float),
            "output_dir": ParameterValue(output_dir, value_type=str),
            "experiment_name": ParameterValue(
                experiment_name, value_type=str
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="my_world.wbt"),
        DeclareLaunchArgument("mode", default_value="realtime"),
        DeclareLaunchArgument("duration", default_value="120.0"),
        DeclareLaunchArgument("output_dir", default_value="results"),
        DeclareLaunchArgument(
            "experiment_name", default_value="noise_0"
        ),
        webots,
        robot_state_publisher,
        footprint_publisher,
        map_publisher,
        supervisor_driver,
        turtlebot_driver,
        waiting_nodes,
        velocity_publisher,
        recorder,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(
                        event=launch.events.Shutdown()
                    )
                ],
            )
        ),
    ])
