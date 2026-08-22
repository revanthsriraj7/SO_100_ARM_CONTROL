import os
from pathlib import Path
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_bridge_config(
    template_file: str, world_name: str, model_name: str, prefix: str = ''
) -> str:
    '''Generate a ros_gz_bridge config file from a template'''
    with open(template_file, 'r') as file:
        config = file.read()

    config = config.replace(
        '{{ world_name }}',
        f'{world_name}',
    )

    config = config.replace(
        '{{ model_name }}',
        f'{model_name}',
    )

    config = config.replace(
        '{{ prefix }}',
        f'{prefix}',
    )

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.yaml')
    temp_file_name = temp_file.name

    with open(temp_file_name, 'w') as temp_file:
        temp_file.write(config)

    return temp_file_name


def get_robot_description(context, *args, **kwargs):
    so_arm_100_description_path = os.path.join(
        get_package_share_directory('so_arm_100_description')
    )

    so_arm_100_moveit_config_path = os.path.join(
        get_package_share_directory('so_arm_100_moveit_config')
    )

    dof = LaunchConfiguration('dof').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    use_topic_hardware_interface = LaunchConfiguration('use_topic_hardware_interface').perform(context)

    xacro_file = os.path.join(
        so_arm_100_description_path,
        'urdf',
        f'so_arm_100_{dof}dof.urdf.xacro',
    )

    ros2_controllers_file = os.path.join(
        so_arm_100_moveit_config_path,
        'config',
        f'controllers_{dof}dof.yaml',
    )

    doc = xacro.process_file(xacro_file, mappings={
        'prefix': prefix,
        'use_sim': 'true',
        'use_topic_hardware_interface': use_topic_hardware_interface,
        'ros2_controllers_file': ros2_controllers_file,
    })

    robot_desc = doc.toprettyxml(indent='  ')

    controller_path = os.path.join(
        so_arm_100_moveit_config_path,
        'config',
        f'controllers_{dof}dof.yaml'
    )
    
    # Convert package:// to model:// for Gazebo
    replace_str = f'package://so_arm_100_description/models/so_arm_100_{dof}dof/meshes'
    with_str = f'model://so_arm_100_{dof}dof/meshes'
    gazebo_robot_desc = robot_desc.replace(replace_str, with_str)

    return {
        'robot_description': ParameterValue(robot_desc, value_type=str),
        'gazebo_description': ParameterValue(gazebo_robot_desc, value_type=str),
        'controller_path': controller_path
    }

def generate_launch_description():
    # Launch Arguments
    so_arm_100_description_path = os.path.join(
        get_package_share_directory('so_arm_100_description')
    )

    so_arm_100_bringup_path = os.path.join(
        get_package_share_directory('so_arm_100_bringup')
    )

    so_arm_100_moveit_config_path = os.path.join(
        get_package_share_directory('so_arm_100_moveit_config')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(so_arm_100_description_path, 'models'),
            ':' + str(Path(so_arm_100_description_path).parent.resolve()),
        ],
    )

    arguments = LaunchDescription([
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
            'use_topic_hardware_interface',
            default_value='false',
            description='Use topic based hardware interface instead of gz_ros_control'
        ),
    ])

    def launch_setup(context, *args, **kwargs):
        dof = LaunchConfiguration('dof').perform(context)
        prefix = LaunchConfiguration('prefix').perform(context)
        world_name = LaunchConfiguration('world').perform(context)
        use_topic = LaunchConfiguration('use_topic_hardware_interface').perform(context) == 'true'
        model_name = 'so_arm_100'

        descriptions = get_robot_description(context)

        robot_desc = descriptions['robot_description']

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='sim',
            output='screen',
            parameters=[
                {'robot_description': robot_desc}
            ]
        )

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
                '/gz_sim.launch.py',
            ]),
            launch_arguments=[
                ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])
            ],
        )

        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=[
                '-string',
                descriptions['gazebo_description'].value,
                '-name',
                'so_arm_100',
                '-allow_renaming',
                'true',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0',
                '-use_sim',
                'true',
            ],
            output='screen'
        )

        # TODO: use dof launch argument consistently

        # ros2_control configuration (since not using gz_ros2_control)
        ros2_control_file = os.path.join(
            so_arm_100_moveit_config_path,
            'config',
            f'controllers_{dof}dof.yaml',
        )

        # controller manager (if not using gz_ros2_control)
        ros2_control_node = Node(
            condition=IfCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package="controller_manager",
            executable="ros2_control_node",
            namespace='sim',
            parameters=[
                ros2_control_file,
                {"use_sim_time": True},
            ],
        )

        # Controller spawner nodes
        # -c targets the sim's namespaced controller_manager (see the
        # <ros><namespace>/sim</namespace></ros> block in
        # so_arm_100_5dof.gazebo.xacro) so this can run alongside a real
        # hardware.launch.py instance without colliding on controller names.
        joint_state_broadcaster_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster', '-c', '/sim/controller_manager'],
            output='screen'
        )

        arm_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'arm_controller', '-c', '/sim/controller_manager'],
            output='screen'
        )

        gripper_controller_spawner = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'gripper_controller', '-c', '/sim/controller_manager'],
            output='screen'
        )

        # ros_gz_bridge when using gz_ros2_control
        # /tf and /tf_static are relative (no leading '/') so the 'sim'
        # namespace prefixes them to /sim/tf, /sim/tf_static, keeping them
        # separate from a real hardware.launch.py instance's /tf. /clock
        # stays global since there's only ever one simulation clock.
        gz_ros2_control_bridge = Node(
            condition=UnlessCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            namespace='sim',
            parameters=[{
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                'tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                'tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                ('/clock', '/clock'),
            ],
        )

        # TODO: rename config file
        bridge_template_file = os.path.join(
            so_arm_100_bringup_path,
            'config',
            'ros_gz_bridge.yaml',
        )

        # Generate bridge config from template
        bridge_config_file = generate_bridge_config(
            bridge_template_file, world_name, model_name, prefix
        )

        # ros_gz_bridge and relay when using topic based control
        topic_based_control_bridge = Node(
            condition=IfCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace='sim',
            parameters=[{"config_file": bridge_config_file}],
            output="screen",
        )

        command_relay = Node(
            condition=IfCondition(LaunchConfiguration('use_topic_hardware_interface')),
            package="so_arm_100_bringup",
            executable=f"so_arm_100_{dof}dof_cmd_relay",
            namespace='sim',
            parameters=[{"prefix": prefix}],
            output="screen",
        )

        nodes = [
            robot_state_publisher,
            gazebo,
            spawn_robot,
            gz_ros2_control_bridge,
            topic_based_control_bridge,
            command_relay,
        ]

        if use_topic:
            # Topic-based control: start ros2_control_node after spawn,
            # then spawn controllers after it starts
            nodes += [
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=spawn_robot,
                        on_exit=[ros2_control_node]
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=ros2_control_node,
                        on_start=[joint_state_broadcaster_spawner]
                    )
                ),
            ]
        else:
            # gz_ros2_control path: spawn controllers after robot is spawned
            nodes.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=spawn_robot,
                        on_exit=[joint_state_broadcaster_spawner]
                    )
                )
            )

        # Common controller spawning chain
        nodes += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_controller_spawner,
                    on_exit=[gripper_controller_spawner]
                )
            ),
        ]
        return nodes

    return LaunchDescription([
        arguments,
        gazebo_resource_path,
        OpaqueFunction(function=launch_setup)
    ])
