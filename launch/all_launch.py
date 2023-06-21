from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Launch `camera_transform.py` launch file
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    current_file_path = os.path.abspath(__file__)
    current_dir_path = os.path.dirname(current_file_path)
    camera_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir_path, '..', 'launch', 'camera_transform.py')
                                      )
    )

    # Launch `camera_launch.py` launch file
    camera_launch_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir_path, '..', 'launch', 'camera_launch.py')
        )
    )
	# Launch `inference_test` node
    inference_test_node = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_markerless_mps', 'inference_test'],
        output='screen'
    )
    
    tf_sniffer_node = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_markerless_mps', 'tf_sniffer'],
        output='screen'
	)
    return LaunchDescription([
        camera_transform_launch,
        camera_launch_launch,
        inference_test_node,
        tf_sniffer_node
    ])
