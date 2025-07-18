from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')
    map_file = LaunchConfiguration('map', default=os.path.join(pkg_nav, 'config', 'model03.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    lifecycle_nodes = ['map_server', 'amcl', 'planner_server', 'controller_server','behavior_server' ,'bt_navigator']

    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', 
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}]
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='nav2_behaviors::BehaviorServer',
                name='behavior_server',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            ),
        ],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params', default_value=params_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        container,
        lifecycle_manager,
    ])
