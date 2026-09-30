from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    kill_default_turtle = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/kill',
                    'turtlesim/srv/Kill',
                    "{name: 'turtle1'}"
                ]
            )
        ]
    )

    spawn_zero = TimerAction(
        period=1.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/spawn',
                    'turtlesim/srv/Spawn',
                    "{x: 1.0, y: 2.0, theta: 1.5708, name: 'turtle_zero'}"
                ]
            )
        ]
    )

    spawn_three = TimerAction(
        period=1.7,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/spawn',
                    'turtlesim/srv/Spawn',
                    "{x: 4.0, y: 2.0, theta: 0.0, name: 'turtle_three'}"
                ]
            )
        ]
    )

    zero_controller = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='turtle_digits',
                executable='digit_controller',
                name='zero_controller',
                parameters=[
                    {'turtle_name': 'turtle_zero'},
                    {'digit': 0}
                ]
            )
        ]
    )

    three_controller = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='turtle_digits',
                executable='digit_controller',
                name='three_controller',
                parameters=[
                    {'turtle_name': 'turtle_three'},
                    {'digit': 3}
                ]
            )
        ]
    )

    return LaunchDescription([
        turtlesim,
        kill_default_turtle,
        spawn_zero,
        spawn_three,
        zero_controller,
        three_controller
    ])
