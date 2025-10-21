#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        LogInfo(msg="[RANDOM_LOCALIZER] Stopping random_localizer_node..."),

        # Kill the random_localizer_node
        ExecuteProcess(
            cmd=['pkill', '-f', 'random_localizer'],
            output='screen'
        ),

        LogInfo(
            msg="[RANDOM_LOCALIZER] Publishing zero velocities continuously for 3 seconds..."),

        # Run Python script that continuously publishes 0 velocity for 3 sec
        # this needs to run on a seperate node so that it can continue running even after this node
        # or package stops, hence the extra lines below
        ExecuteProcess(
            cmd=[
                'python3', '-c',
                """
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

rclpy.init()
node = Node('stop_robot_node')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

msg = Twist()
msg.linear.x = 0.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0

start_time = time.time()
while time.time() - start_time < 3.0:
    pub.publish(msg)
    node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # 10 Hz

node.destroy_node()
rclpy.shutdown()
"""
            ],
            output='screen'
        )
    ])
