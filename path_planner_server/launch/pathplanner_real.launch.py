import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    target_arg = DeclareLaunchArgument(
                'target', default_value='real'
                 )
    run_env_f  = LaunchConfiguration('target')

    real_planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real/planner_server.yaml')
    real_controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real/controller.yaml')
    real_bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real/bt.yaml')
    real_recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real/recovery.yaml')
    real_filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real/filters.yaml')
    rviz_file = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'real_pathplanning.rviz')

    mask_yaml_file = os.path.join(get_package_share_directory('map_server'), 'config', 'real_cafe_r2e_keepout_mask.yaml')

    # Make re-written yaml
    fm_param_substitutions = {
        'yaml_filename': mask_yaml_file}

    fm_configured_params = RewrittenYaml(
        source_file=real_filters_yaml,
        root_key='',
        param_rewrites=fm_param_substitutions,
        convert_types=True)

    real_filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        # parameters=[real_filters_yaml],
        parameters=[fm_configured_params],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
            ('/scan', '/cleaner_2/scan'),
            ('/odom', '/cleaner_2/odom'),
        ],
    )

    real_costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        # parameters=[real_filters_yaml],
        parameters=[fm_configured_params],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
            ('/scan', '/cleaner_2/scan'),
            ('/odom', '/cleaner_2/odom'),
        ],
    )

    real_nav2_planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[real_planner_yaml],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
            ('/scan', '/cleaner_2/scan'),
            ('/odom', '/cleaner_2/odom'),
        ],
    )

    real_nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[real_controller_yaml],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
            ('/scan', '/cleaner_2/scan'),
            ('/odom', '/cleaner_2/odom'),
            ('/cmd_vel', '/cleaner_2/cmd_vel'),
        ],
    )

    bt_xml_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'real/navigate_w_replanning_and_recovery.xml')

    # Make re-written yaml
    bt_param_substitutions = {
        'default_nav_to_pose_bt_xml': bt_xml_file}

    bt_configured_params = RewrittenYaml(
        source_file=real_bt_navigator_yaml,
        root_key='',
        param_rewrites=bt_param_substitutions,
        convert_types=True)

    real_nav2_bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        # parameters=[real_bt_navigator_yaml],
        parameters=[bt_configured_params],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
            ('/scan', '/cleaner_2/scan'),
            ('/odom', '/cleaner_2/odom'),
        ],
    )

    real_nav2_recoveries_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[real_recovery_yaml],
        remappings=[
            ('/tf', '/cleaner_2/tf'),
            ('/tf_static', '/cleaner_2/tf_static' ),
            ('/scan', '/cleaner_2/scan'),
            ('/odom', '/cleaner_2/odom'),
        ],
    )

    real_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_pathplanner',
        parameters=[{'autostart': True},
                    {'bond_timeout':0.0},
                    {'node_names': ['planner_server', 
                                    'controller_server', 
                                    'bt_navigator', 
                                    'behavior_server',
                                    'filter_mask_server',
                                    'costmap_filter_info_server'
                                    ]}
                    ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d ", rviz_file]
    )

    return LaunchDescription(
        [
            target_arg,
            # rviz_node,
            GroupAction(
                condition=LaunchConfigurationEquals('target', 'real'),
                actions = [
                    real_nav2_planner_node,
                    real_nav2_controller_node,
                    real_nav2_bt_navigator_node,
                    real_nav2_recoveries_node,
                    real_filter_mask_server_node,
                    real_costmap_filter_info_server_node,
                    real_lifecycle_manager_node
                ]
            )

        ]
    )