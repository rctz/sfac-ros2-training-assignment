import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_data_processor',
            executable='sensor_processor',
            name='sensor_processor',
            output='screen',
        ),

        Node(
            package='robot_sensor_publisher',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen',
        ),
    ])