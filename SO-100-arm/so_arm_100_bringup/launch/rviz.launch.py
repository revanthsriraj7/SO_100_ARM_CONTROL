import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def get_robot_description(context, *args, **kwargs):
    so_arm_100_description_path = os.path.join(
        get_package_share_directory('so_arm_100_description')
    )

    rviz_config_file = os.path.join(
        so_arm_100_description_path,
        'rviz',
        'so_arm_100.rviz'
    )
    
    return {
        'rviz_config_file': rviz_config_file,
    }

def generate_launch_description():
    def launch_setup(context, *args, **kwargs):
        params = get_robot_description(context)
        
        nodes = [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', params['rviz_config_file']]
            )
        ]
        return nodes

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
