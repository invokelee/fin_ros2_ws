from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pub_node = Node(
        package='attach_shelf',
        executable='publisher_member_function_node',
        output='screen',
        name='publisher_member_function_node',
    )

    sub_node = Node(
        package='attach_shelf',
        executable='subscriber_member_function_node',
        output='screen',
        name='subscriber_member_function_node',
    )

    return LaunchDescription(
        [
            # pub_node,
            sub_node
        ]
    )