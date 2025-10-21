#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction

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
            period=2.0,  # optional small delay before launching
            actions=[
                LogInfo(msg="[RANDOM_LOCALIZER] Launching random_localizer node..."),
                random_localizer_node
            ]
        ),
    ])

