from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_arguments = [
        DeclareLaunchArgument(
            'x',
            default_value='-0.05',
            description='The x-axis value of the static transform.'),
        DeclareLaunchArgument(
            'y',
            default_value='0.05',
            description='The y-axis value of the static transform.'),
        DeclareLaunchArgument(
            'z',
            default_value='0.3',
            description='The z-axis value of the static transform.'),
        DeclareLaunchArgument(
            'roll',
            default_value='-0.959931',
            description='The roll angle of the static transform.'),
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='The pitch angle of the static transform.'),
        DeclareLaunchArgument(
            'yaw',
            default_value='-1.74533',
            description='The yaw angle of the static transform.'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='robotinobase1plate_top',
            description='The parent frame ID for the static transform.'),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='camera_link',
            description='The child frame ID for the static transform.')
    ]

    # Define the static transform publisher node
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            LaunchConfiguration('x'),
            LaunchConfiguration('y'),
            LaunchConfiguration('z'),
            LaunchConfiguration('roll'),
            LaunchConfiguration('pitch'),
            LaunchConfiguration('yaw'),
            LaunchConfiguration('frame_id'),
            LaunchConfiguration('child_frame_id')
        ],
        output='screen'
    )

    return LaunchDescription(declare_arguments + [static_transform_publisher])
