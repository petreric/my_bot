from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Define paths to the different files and packages
    pkg_my_bot = FindPackageShare(package='my_bot').find('my_bot')
    world_file = PathJoinSubstitution([pkg_my_bot, 'worlds', 'obstacles.world'])
    rviz_config_file = PathJoinSubstitution([pkg_my_bot, 'config', 'navigation.rviz'])
    slam_params_file = PathJoinSubstitution([pkg_my_bot, 'config', 'mapper_params_online_async.yaml'])

    # Include the launch file to start the simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_my_bot, 'launch', 'launch_sim.launch.py'])
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Launch RViz2 with the specified configuration file
    rviz2_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    # Launch SLAM Toolbox with specified parameters
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package='slam_toolbox'), 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        sim_launch,
        rviz2_node,
        slam_toolbox_launch
    ])
