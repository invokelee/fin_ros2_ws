import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    rviz_file = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'mapper_rviz_config.rviz')

    configuration_basename  = 'cartographer_sim.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

# for testing
    base_laser_sensor_link_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        name='base_laser_broadcaster_node',
        # namespace='/cleaner_2',
        parameters=[{'use_sim_time': True}],
        arguments=['0.103007', '0', '0.17', '0', '0',
                       '0', '1', 'robot_base_link', 'laser_sensor_link'],
        # remappings=[
        #     ('/tf', '/cleaner_2/tf'),
        #     ('/tf_static', '/cleaner_2/tf_static' ),
        # ],
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
            rviz_node,
            # base_laser_sensor_link_tf_publisher_node,
            cartographer_node,
            occupancy_grid_node
        ]
    )