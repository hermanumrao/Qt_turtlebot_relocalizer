#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, LogInfo, SetEnvironmentVariable

def generate_launch_description():
    # Set TurtleBot3 model
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')

    # Gazebo launch (immediate)
    gazebo_cmd = [
        'ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'
    ]

    # Navigation2 launch (delayed 10s to ensure Gazebo is ready)
    home = os.environ['HOME']
    map_file = os.path.join(home, 'maps', 'map1.yaml')
    nav2_cmd = [
        'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
        'use_sim_time:=True',
        f'map:={map_file}'
    ]

    return LaunchDescription([
        set_model,

        LogInfo(msg="[TURTLEBOT3_SIM] Launching Gazebo..."),
        ExecuteProcess(cmd=gazebo_cmd, output='screen', shell=True),

        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="[TURTLEBOT3_SIM] Launching Navigation2..."),
                ExecuteProcess(cmd=nav2_cmd, output='screen', shell=True)
            ]
        ),
    ])

