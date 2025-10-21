#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
# this is used to luanch random_localizer package which will send random
# cmd_velocities to the turtlebot so that it can move while avoiding obstacles


def generate_launch_description():
    random_localizer_node = Node(
        package='random_localizer',
        executable='random_localizer',
        name='random_localizer_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        TimerAction(
            period=2.0,  # small delay, to avoid clashes
            actions=[
                LogInfo(
                    msg="[RANDOM_LOCALIZER] Launching random_localizer node..."),
                random_localizer_node
            ]
        ),
    ])
