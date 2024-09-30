from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.3"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90"
    )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="true"
    )

    run_env_arg = DeclareLaunchArgument(
        "target", default_value="sim"
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')
    run_env_f  = LaunchConfiguration('target')

    pre_approach_v2_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2_node',
        parameters=[{
            'obstacle': obstacle_f,
            'degrees': degrees_f,
            'final_approach': final_approach_f
        }]
    )

    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server_node',
        parameters=[{
            'target': run_env_f
        }]

    )

    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            run_env_arg,
            # pre_approach_v2_node,
            approach_service_server_node
        ]
    )