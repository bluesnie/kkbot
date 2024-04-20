import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

# def generate_launch_description():

#     kkbot_navigation2_dir = get_package_share_directory('kkbot_navigation2')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     map_yaml_path = LaunchConfiguration('map', default=os.path.join(kkbot_navigation2_dir, 'map', 'turtlebot3_world.yaml'))
#     # map_yaml_path = LaunchConfiguration('map', default=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'))
#     nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(kkbot_navigation2_dir, 'param', 'kkbot_nav2.yaml'))
#     rviz_config_dir = os.path.join(kkbot_navigation2_dir, 'rviz', 'kkbot_rviz2.rviz')

#     nav2_bringup_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
#         launch_arguments={
#             'map': map_yaml_path,
#             'use_sim_time': use_sim_time,
#             'params_file': nav2_param_path
#         }.items()
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', rviz_config_dir],
#         parameters=[{'use_sim_time': use_sim_time}],
#         output='screen'
#     )
#     return LaunchDescription([nav2_bringup_launch, rviz_node])


def generate_launch_description():

    kkbot_navigation2_dir = get_package_share_directory('kkbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    
    namespace = LaunchConfiguration('namespace', default="")
    autostart = LaunchConfiguration('autostart', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(kkbot_navigation2_dir, 'map', 'turtlebot3_world.yaml'))
    # map_yaml_path = LaunchConfiguration('map', default=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(kkbot_navigation2_dir, 'param', 'kkbot_nav2.yaml'))
    mask_yaml_file = LaunchConfiguration('mask',default=os.path.join(kkbot_navigation2_dir, 'map', 'keepout_mask_turtlebot3_world.yaml'))
    rviz_config_dir = os.path.join(kkbot_navigation2_dir, 'rviz', 'kkbot_rviz2.rviz')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file}
    configured_params = RewrittenYaml(
        source_file=nav2_param_path,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items()
        ),
        Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[configured_params]),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
    ])

    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)
    return ld

