"""Standalone launch for just the OminiBotHV chassis driver.

  ros2 launch ominibot_driver ominibot_driver.launch.py
  ros2 launch ominibot_driver ominibot_driver.launch.py port:=/dev/ttyS0

Normally you don't run this directly -- robot_bringup.launch.py in
car_assemble_description includes this node. Handy for bench-testing the base alone.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial0',
                              description='serial device of the OminiBotHV board (Pi GPIO UART)'),
        Node(
            package='ominibot_driver',
            executable='ominibot_driver_node',
            name='ominibot_driver',
            output='screen',
            parameters=[{'port': port}],
        ),
    ])
