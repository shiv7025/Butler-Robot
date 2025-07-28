# File: butler_robot_ws/src/bumperbot_description/launch/butler.launch.py
# Description: This is the final, top-level launch file to run the entire project.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bumperbot_description = get_package_share_directory('bumperbot_description')

    # --- Include the Navigation Launch File ---
    # This starts Gazebo, Nav2, and RViz
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bumperbot_description, 'launch', 'navigation.launch.py')
        )
    )

    # --- Define the Butler Logic Node ---
    # This starts your action server script
    butler_logic_node = Node(
        package='bumperbot_description',
        executable='butler_node.py',
        name='butler_action_server',
        output='screen'
    )

    # --- Create the Final Launch Description ---
    return LaunchDescription([
        navigation_launch,
        butler_logic_node
    ])
