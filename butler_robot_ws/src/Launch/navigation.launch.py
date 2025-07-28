# File: butler_robot_ws/src/bumperbot_description/launch/navigation.launch.py
# Description: This is the top-level launch file to start the full navigation stack.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the share directory of the bumperbot_description package
    pkg_bumperbot_description = get_package_share_directory('bumperbot_description')

    # --- Declare Launch Arguments ---

    # Path to the map file for Nav2
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bumperbot_description, 'maps', 'restaurent_map.yaml'),
        description='Full path to map file to load'
    )

    # Path to the Nav2 parameters file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_bumperbot_description, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Whether to use simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_bumperbot_description, 'rviz', 'nav2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    # --- Include other Launch Files ---

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bumperbot_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world_name': 'kitchen'}.items()  # UPDATED LINE
    )

    # Include the Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # --- Define Nodes ---

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # --- Create Launch Description ---

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_file_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_config_arg)

    # Add included launch files
    ld.add_action(gazebo_launch)
    ld.add_action(nav2_bringup_launch)

    # Add nodes
    ld.add_action(rviz_node)

    return ld
