import os

from ament_index_python.packages import get_package_share_directory
from launch.action import IncludeLaunchDescription
from launch.launch_description_source import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    # cur_pkg_dir = get_package_share_directory("kkbot_simple_demo")
    kkbot_nav2_dir  = get_package_share_directory("kkbot_navigation2")

    kkbot_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kkbot_nav2_dir, 'launch', 'tb3_simulation_launch.py')
            )
        )
    demo_cmd = Node(
        package='kkbot_simple_demo',
        executable="example_nav_to_pose",
        emulate_tty=True,
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(kkbot_nav2_cmd)
    ld.add_action(demo_cmd)
    return ld