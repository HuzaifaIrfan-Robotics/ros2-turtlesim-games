from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )
    ld.add_action(turtlesim_node)

    # Launch spawner node
    spawner_node = Node(
        package='turtlesim_catch_them_all',
        executable='spawner',
    )
    ld.add_action(spawner_node)

    controller_node = Node(
        package='turtlesim_catch_them_all',
        executable='controller',
        namespace='player1',
    )
    ld.add_action(controller_node)

    controller_node = Node(
        package='turtlesim_catch_them_all',
        executable='controller',
        namespace='player2',
    )
    ld.add_action(controller_node)

    # controller_node = Node(
    #     package='turtlesim_catch_them_all',
    #     executable='controller',
    #     namespace='master3',
    # )
    # ld.add_action(controller_node)

    return ld
