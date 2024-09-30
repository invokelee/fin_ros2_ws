import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='warehouse_map_sim.yaml'
    )
    map_file_f = LaunchConfiguration('map_file')

    map_file_path  = PathJoinSubstitution([get_package_share_directory('map_server'), 'config', map_file_f])
    rviz_file = os.path.join(get_package_share_directory('map_server'), 'rviz', 'mapper_rviz_config.rviz')

    real_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
                    {'yaml_filename' : map_file_path}
                   ]
    )
    
    sim_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaml_filename' : map_file_path}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_file]
    )

    real_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_mapper',
        parameters=[
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    sim_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_mapper',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    return LaunchDescription(
        [
            map_file_arg,
            rviz_node,
            GroupAction(
                condition=LaunchConfigurationEquals('map_file', 'warehouse_map_sim.yaml'),
                actions = [
                    sim_map_server_node,
                    sim_lifecycle_manager_node
                ]
            ),
            GroupAction(
                condition=LaunchConfigurationEquals('map_file', 'warehouse_map_real.yaml'),
                actions = [
                    real_map_server_node,
                    real_lifecycle_manager_node
                ]
            )
        ]
    )