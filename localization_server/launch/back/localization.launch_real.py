import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='warehouse_map_sim.yaml'
    )
    map_file_f = LaunchConfiguration('map_file')

    map_file_path  = PathJoinSubstitution([get_package_share_directory('map_server'), 'config', map_file_f])
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    rviz_file = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
                    {'yaml_filename' : map_file_path}
                   ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_localization',
        parameters=[
                    {'autostart': True},
                    {'bond_timeout':0.0},
                    {'node_names': ['map_server', 'amcl']}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_file]
    )

    return LaunchDescription(
        [
            # rviz_node,
            map_file_arg,
            map_server_node,
            amcl_node,
            lifecycle_manager_node
        ]
    )