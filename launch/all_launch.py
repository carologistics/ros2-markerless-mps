from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch `camera_transform.py` launch file
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    camera_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/robotino/ros2_honies_ws/src/ros2-markerless-mps/launch/camera_transform.py'
                                      )
    )

    # Launch `camera_launch.py` launch file
    camera_launch_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/robotino/ros2_honies_ws/src/hthesis3/launch/camera_launch.py'
        )
    )
	# Launch `inference_test` node
    inference_test_node = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2-markerless-mps', 'inference_test'],
        output='screen'
    )
    
    tf_sniffer_node = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2-markerless-mps', 'tf_sniffer'],
        output='screen'
	)
    return LaunchDescription([
        camera_transform_launch,
        camera_launch_launch,
        inference_test_node,
        tf_sniffer_node
    ])
