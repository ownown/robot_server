import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# https://github.com/olmerg/lesson_urdf/blob/master/launch/view_robot_launch.py

def generate_launch_description():
    robot_dir = get_package_share_directory('robot_server')
    ld = LaunchDescription()

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(robot_dir, 'rviz', 'view.rviz'),
        description='Path to the RViz config file'
    )

    server_node = Node(
        package='robot_server',
        executable='server',
        name='server'
    )

    viz_node = Node(
        package='robot_server',
        executable='visualization',
        name='viz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file]
    )

    frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_frame_publisher',
        args=[]
    )

    # ld.add_action(rviz_config_file_cmd)

    ld.add_action(server_node)
    ld.add_action(viz_node)
    ld.add_action(rviz)

    return ld
