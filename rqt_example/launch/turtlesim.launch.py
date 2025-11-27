from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Execute the rqt_example with turtlesim node.']),

        Node(
            namespace='turtle1',
            package='rqt_example',
            executable='rqt_example',
            name='rqt_example',
            output='screen'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen')
    ])
