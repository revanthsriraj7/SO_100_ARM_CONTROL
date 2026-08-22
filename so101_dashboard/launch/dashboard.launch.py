"""Launches the SO-101 dashboard.

    ros2 launch so101_dashboard dashboard.launch.py
    ros2 launch so101_dashboard dashboard.launch.py mock:=true

Prefer running this alongside hardware.launch.py (from so_arm_100_bringup)
for the real robot -- this launch file does not start the robot's own
controllers/hardware interface, it only starts the dashboard GUI process.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mock_arg = DeclareLaunchArgument(
        'mock', default_value='false',
        description='Run the dashboard against a synthetic joint-state generator, no real robot needed.',
    )

    real_node = Node(
        condition=UnlessCondition(LaunchConfiguration('mock')),
        package='so101_dashboard',
        executable='dashboard',
        name='so101_dashboard',
        output='screen',
    )

    mock_node = Node(
        condition=IfCondition(LaunchConfiguration('mock')),
        package='so101_dashboard',
        executable='dashboard',
        name='so101_dashboard',
        arguments=['--mock'],
        output='screen',
    )

    return LaunchDescription([mock_arg, real_node, mock_node])
