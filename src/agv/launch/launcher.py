#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():
    # Launch configuration variables
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')
    joy_vel = launch.substitutions.LaunchConfiguration('joy_vel')

    # Create the launch description object
    ld = launch.LaunchDescription()

    # Declare launch arguments
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            'joy_vel',
            default_value='cmd_vel',
            description='Topic on which to publish velocity commands'
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            'joy_config',
            default_value='ps3',
            description='Configuration file to use (ps3, xbox, etc.)'
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Device file corresponding to the joystick'
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            'config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(
                    text=os.path.join(
                        get_package_share_directory('teleop_twist_joy'),
                        'config', ''
                    )
                ),
                joy_config,
                launch.substitutions.TextSubstitution(text='.config.yaml')
            ],
            description='Full path to the YAML configuration file for teleop_twist_joy'
        )
    )

    # Joy Node
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
 
    # Teleop Twist Joy Node
    teleop_twist_joy_node = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        remappings=[
            ('/cmd_vel', joy_vel)
        ],
    )

    # ODrive Controller Node (cmd_vel_to_odrive)
    odrive_node = launch_ros.actions.Node(
        package='agv_controller',
        executable='cmd_vel_to_odrive',  # Must match the entry point defined in your setup.py
        name='cmd_vel_to_odrive',
        output='screen'
    )

    # Launch the odrive_can example launch file as a separate process
    odrive_can_launch = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'odrive_can', 'example_launch.yaml'],
        output='screen'
    )

    # Add all nodes and the execute process to the launch description
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(odrive_node)
    ld.add_action(odrive_can_launch)

    return ld
