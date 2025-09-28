from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    # Launch teleop_key node
    teleop_key_node = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='teleop',
        prefix='xterm -e',   # ensures teleop runs in a new terminal
        respawn=True,             # <-- will auto-restart if it crashes
        respawn_delay=2.0
    )

    ld.add_action(turtlesim_node)
    ld.add_action(teleop_key_node)

    return ld
