import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration('map')
    param_file_name = LaunchConfiguration('param_file_name')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(get_package_share_directory('my_bot'), 'maps', 'map.yaml'),
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('my_bot'), 'config', 'nav2_params.yaml'),
            description='Full path to param file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RVIZ config file'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_file_name
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
