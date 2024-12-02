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
                'target', default_value='sim'
                 )
    run_env_f  = LaunchConfiguration('target')

    sim_planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim/planner_server.yaml')
    sim_controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim/controller.yaml')
    sim_bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim/bt.yaml')
    sim_recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim/recovery.yaml')
    sim_filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim/filters.yaml')
    sim_rviz_file = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'sim_pathplanning.rviz')

    mask_yaml_file = os.path.join(get_package_share_directory('map_server'), 'config', 'sim_cafe_r3e_keepout_mask_map.yaml')

    # Make re-written yaml
    fm_param_substitutions = {
        'yaml_filename': mask_yaml_file}

    fm_configured_params = RewrittenYaml(
        source_file=sim_filters_yaml,
        root_key='',
        param_rewrites=fm_param_substitutions,
        convert_types=True)

    sim_filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        # parameters=[sim_filters_yaml]
        parameters=[fm_configured_params]
    )

    sim_costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        # parameters=[sim_filters_yaml]
        parameters=[fm_configured_params]
    )

    sim_nav2_planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[sim_planner_yaml]
    )

    sim_nav2_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[sim_controller_yaml],
        remappings=[('cmd_vel', 'diffbot_base_controller/cmd_vel_unstamped')]
    )

    bt_xml_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim/navigate_w_replanning_and_recovery.xml')

    # Make re-written yaml
    bt_param_substitutions = {
        'default_nav_to_pose_bt_xml': bt_xml_file}

    bt_configured_params = RewrittenYaml(
        source_file=sim_bt_navigator_yaml,
        root_key='',
        param_rewrites=bt_param_substitutions,
        convert_types=True)

    sim_nav2_bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        # parameters=[sim_bt_navigator_yaml]
        parameters=[bt_configured_params]
    )

    sim_nav2_recoveries_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[sim_recovery_yaml],
        remappings=[('cmd_vel', 'diffbot_base_controller/cmd_vel_unstamped')]
    )

    sim_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        name='lifecycle_manager_pathplanner',
        parameters=[
                    {'use_sim_time': True},
                    {'autostart': True},
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
        arguments=["-d ", sim_rviz_file]
    )

    return LaunchDescription(
        [
            target_arg,
            # rviz_node,
            GroupAction(
                condition=LaunchConfigurationEquals('target', 'sim'),
                actions = [
                    sim_nav2_planner_node,
                    sim_nav2_controller_node,
                    sim_nav2_bt_navigator_node,
                    sim_nav2_recoveries_node,
                    sim_filter_mask_server_node,
                    sim_costmap_filter_info_server_node,
                    sim_lifecycle_manager_node
                ]
            )
        ]
    )