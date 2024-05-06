import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    warehouse_dir = get_package_share_directory("aws_robomaker_small_warehouse_world")
    kkbot_nav2_dir  = get_package_share_directory("kkbot_navigation2")
    kkbot_desc_dir  = get_package_share_directory("kkbot_description")
    simple_demo_dir = get_package_share_directory("kkbot_simple_demo")

    map_yaml_file = os.path.join(warehouse_dir, 'maps', '005', 'map.yaml')
    world = os.path.join(simple_demo_dir, 'warehouse.world')

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient)')

    # start the simulation
    # 如果出现这个进程起不来，查看是否还存在执行的gzserver进程，kill再起
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[warehouse_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', headless])),
        cmd=['gzclient'],
        cwd=[warehouse_dir], output='screen')


    urdf = os.path.join(kkbot_desc_dir, 'urdf', 'turtlebot3_waffle.urdf')
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf])
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kkbot_nav2_dir, 'launch', 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={"namespace": "", "use_namespace": "False"}.items())

    kkbot_nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kkbot_nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={"map": map_yaml_file}.items())
    
    demo_cmd = Node(
        package='kkbot_simple_demo',
        executable="example_nav_to_pose",
        emulate_tty=True,
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(kkbot_nav2_bringup_cmd)
    ld.add_action(demo_cmd)
    return ld