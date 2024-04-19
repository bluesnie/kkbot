import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_name_in_model = "kkbot"
    package_name = "kkbot_description"
    urdf_name = "kkbot_base.urdf"

    ld = LaunchDescription()

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.00'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00')}

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',  os.path.join(pkg_share, "world/kkbot.world")],
        output="screen"
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path, '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']], output="screen"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path]
    )
    # rviz2_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen"
    # )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(rviz2_node)

    return ld