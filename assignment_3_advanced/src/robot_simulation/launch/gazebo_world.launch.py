import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    robot_description_path = os.path.join(
        get_package_share_directory('robot_simulation'), 'urdf', 'my_robot.urdf.xacro')

    from xacro import process_file
    robot_description_config = process_file(robot_description_path)

    robot_description = {'robot_description': robot_description_config.toxml()}

    world_file_name = 'turtlebot3_house.world'
    world_path = PathJoinSubstitution([
        turtlebot3_gazebo_dir,
        'worlds',
        world_file_name
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    # send data from urdf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            # 'robot_description': robot_description,
            robot_description,
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )


    robot_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',  
        arguments=[
            '-topic', 'robot_description',  
            '-entity', 'my_robot',  
            '-x', '0.0',  
            '-y', '-2.0',  
            '-z', '0',
            '-Y', '3.14',
        ],
        output='screen'
    )

    return LaunchDescription([   
        gazebo_launch,  
        robot_state_publisher,
        joint_state_publisher,
        robot_spawn,   
        LogInfo(
            condition=None,
            msg="Launching Gazebo and Robot Description Complete"
        )
    ])
