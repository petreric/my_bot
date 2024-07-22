from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the package path
    pkg_my_bot = FindPackageShare('my_bot').find('my_bot')

    # Declare the launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='src/my_bot/worlds/obstacles.world',
        description='Path to the world file'
    )
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value='src/my_bot/maps/my_map_save.yaml',
        description='Path to the map file'
    )
    declare_param_file_name_arg = DeclareLaunchArgument(
        'param_file_name',
        default_value='src/my_bot/config/nav2_params.yaml',
        description='Path to the navigation parameters file'
    )
    declare_rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='src/my_bot/config/navigation.rviz',
        description='Path to the RViz configuration file'
    )

    # Include the simulation launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_my_bot, 'launch', 'launch_sim.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Include the navigation launch file
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_my_bot, 'launch', 'navigation2.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'param_file_name': LaunchConfiguration('param_file_name'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file')
        }.items()
    )

    return LaunchDescription([
        declare_world_arg,
        declare_map_arg,
        declare_param_file_name_arg,
        declare_rviz_config_file_arg,
        sim_launch,
        nav_launch
    ])
