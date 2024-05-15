from launch import LaunchDescription
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    simple_bot_bringup_path = get_package_share_directory('simple_bot_bringup')

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(simple_bot_bringup_path, 'rviz', 'config.rviz')],
        output="screen",
    )

    xacro_file = os.path.join(simple_bot_bringup_path, 'urdf', 'simple_bot.xacro')
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': Command(['xacro ', xacro_file]) }
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            rviz2
        ])