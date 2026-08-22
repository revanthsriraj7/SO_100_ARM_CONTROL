"""Digital twin: sim (namespaced /sim) mirroring a real robot's joint states.

Run this ALONGSIDE hardware.launch.py, not instead of it:

    # terminal 1
    ros2 launch so_arm_100_bringup hardware.launch.py
    # terminal 2
    ros2 launch so_arm_100_bringup digital_twin.launch.py

The sim's controller_manager/controllers/tf live under /sim (see the
<ros><namespace>/sim</namespace></ros> block in
so_arm_100_5dof.gazebo.xacro) so they don't collide with the real robot's
identically-named topics. real_to_sim_bridge.py reads the real robot's
/joint_states and drives the sim's controllers to match -- one direction
only, so nothing here can command the physical arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    so_arm_100_bringup_path = get_package_share_directory('so_arm_100_bringup')

    arguments = [
        DeclareLaunchArgument(
            'dof',
            default_value='5',
            description='DOF configuration - either 5 or 7'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of joint and link names'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Gz sim World'
        ),
        DeclareLaunchArgument(
            'real_joint_states_topic',
            default_value='/joint_states',
            description="Topic hardware.launch.py's joint_state_broadcaster publishes on"
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Rate (Hz) at which the real state is mirrored into the sim'
        ),
    ]

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(so_arm_100_bringup_path, 'launch', 'gz.launch.py')
        ]),
        launch_arguments=[
            ('dof', LaunchConfiguration('dof')),
            ('prefix', LaunchConfiguration('prefix')),
            ('world', LaunchConfiguration('world')),
        ],
    )

    mirror = Node(
        package='so_arm_100_bringup',
        executable='real_to_sim_bridge',
        name='real_to_sim_bridge',
        output='screen',
        remappings=[
            ('joint_states', LaunchConfiguration('real_joint_states_topic')),
        ],
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
    )

    return LaunchDescription(arguments + [gz, mirror])
