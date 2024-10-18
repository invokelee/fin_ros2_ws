import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='real_cafe_area.yaml'
    )
    map_file_f = LaunchConfiguration('map_file')

    map_file_path  = PathJoinSubstitution([get_package_share_directory('cartographer_slam'), 'config', map_file_f])
    nav2_yaml_real = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')
    rviz_file_real = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'real_pathplanning.rviz')
    # rviz_file_real = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'real_localizer_rviz_config.rviz')

    real_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
                    {'use_sim_time': False},
                    {'yaml_filename' : map_file_path}
                   ]
    )

    real_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml_real],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
        ],

    )

    real_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_localization',
        parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'bond_timeout':0.0},
                    {'node_names': ['map_server', 'amcl']}]
    )

    real_base_laser_sensor_link_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        name='base_laser_broadcaster_node',
        # namespace='/cleaner_2',
        parameters=[{'use_sim_time': False}],
        arguments=['0.103007', '0', '0.17', '0', '0',
                       '0', '1', 'base_link', 'laser_sensor_link'],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
        ],
    )

    real_base_laser_sensor_link_tf_publisher_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        name='base_laser_broadcaster_node',
        # namespace='/cleaner_2',
        parameters=[{'use_sim_time': False}],
        arguments=['0.103007', '0', '0.17', '0', '0',
                       '0', '1', 'base_link', 'cleaner_2/laser_sensor_link'],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
        ],
    )

    rviz_node_real = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_file_real],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
        ],
    )

    return LaunchDescription(
        [
            map_file_arg,
            rviz_node_real,
            # real_base_laser_sensor_link_tf_publisher_node,
            real_base_laser_sensor_link_tf_publisher_node2,
            GroupAction(
                condition=LaunchConfigurationEquals('map_file', 'real_cafe_area.yaml'),
                actions = [
                    real_map_server_node,
                    real_amcl_node,
                    real_lifecycle_manager_node
                ]
            )
        ]
    )