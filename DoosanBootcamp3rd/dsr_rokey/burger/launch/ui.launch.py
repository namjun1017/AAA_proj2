import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # RQT example node
    rqt_node = Node(
        package='rqt_example',
        executable='rqt_example',
        name='rqt_example',
        output='screen'
    )

    # Get order node
    get_order_node = Node(
        package='get_order',
        executable='get_order',
        name='get_order',
        output='screen'
    )

    # Order details node
    order_details_node = Node(
        package='order_details',
        executable='order_details',
        name='order_details',
        output='screen'
    )

    return LaunchDescription([
        #dsr_launch,
        #realsense_launch,
        # detection_node,
        # robot_move_node,
        rqt_node,
        get_order_node,
        order_details_node
    ])

