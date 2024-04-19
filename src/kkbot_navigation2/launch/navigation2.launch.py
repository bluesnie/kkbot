import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    kkbot_navigation2_dir = get_package_share_directory('kkbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # map_yaml_path = LaunchConfiguration('map', default=os.path.join(kkbot_navigation2_dir, 'map', 'kkbot_map.yaml'))
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(kkbot_navigation2_dir, 'param', 'kkbot_nav2.yaml'))
    rviz_config_dir = os.path.join(kkbot_navigation2_dir, 'rviz', 'kkbot_rviz2.rviz')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    return LaunchDescription([nav2_bringup_launch, rviz_node])