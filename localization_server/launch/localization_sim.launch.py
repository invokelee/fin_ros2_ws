import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='sim_cafe_area.yaml'
    )
    map_file_f = LaunchConfiguration('map_file')

    map_file_path  = PathJoinSubstitution([get_package_share_directory('cartographer_slam'), 'config', map_file_f])
    nav2_yaml_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    rviz_file_sim = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'sim_pathplanning.rviz')

    sim_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaml_filename' : map_file_path}]
    )

    sim_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml_sim]
    )

    sim_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'amcl']}]
    )

    rviz_node_sim = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_file_sim]
    )

    return LaunchDescription(
        [
            map_file_arg,
            rviz_node_sim,
            GroupAction(
                condition=LaunchConfigurationEquals('map_file', 'sim_cafe_area.yaml'),
                actions = [
                    sim_map_server_node,
                    sim_amcl_node,
                    sim_lifecycle_manager_node
                ]
            ),
        ]
    )