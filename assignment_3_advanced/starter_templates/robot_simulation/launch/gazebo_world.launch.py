from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # TODO: Configure package and world paths
    # pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    # pkg_turtlebot3_gazebo = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')

    # TODO: Set world file path
    # world_file_name = 'your_custom_world.world'
    # world_path = PathJoinSubstitution([
    #     FindPackageShare('robot_simulation'),
    #     'worlds',
    #     world_file_name
    # ])

    # TODO: Configure Gazebo launch
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gazebo.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'world': world_path,
    #         'verbose': 'true'
    #     }.items()
    # )

    # TODO: Configure robot spawn
    # robot_spawn = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('turtlebot3_gazebo'),
    #             'launch',
    #             'spawn_turtlebot3.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'x_pose': '0.0',
    #         'y_pose': '0.0',
    #         'z_pose': '0.1'
    #     }.items()
    # )

    return LaunchDescription([
        # Add your launch actions here
        # gazebo_launch,
        # robot_spawn,
    ])
