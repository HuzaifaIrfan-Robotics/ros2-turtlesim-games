from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Turtlesim GUI node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Joystick driver node (reads /dev/input/js0) x-input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',   # your joystick device
                'deadzone': 0.1,           # ignore tiny stick movements
                'autorepeat_rate': 20.0    # rate for repeating button events
            }]
        ),

        # Teleop Twist Joy (maps joystick -> velocity commands)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,         # left stick vertical
                'scale_linear.x': 2.0,
                'axis_angular.yaw': 3,      # right stick horizontal
                'scale_angular.yaw': 2.0,
                'enable_button': 4,         # joystick left back button enable movement
                'enable_turbo_button': 5    # joystick right back button slow mode
            }],
            remappings=[('/cmd_vel', '/turtle1/cmd_vel')]
        )

    ])
