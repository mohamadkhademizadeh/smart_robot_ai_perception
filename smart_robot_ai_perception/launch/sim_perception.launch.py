from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_robot_ai_perception')
    params = os.path.join(pkg_share, 'config', 'params.yaml')

    detector = Node(
        package='smart_robot_ai_perception',
        executable='detector_node',
        name='detector_node',
        parameters=[params, {'backend': LaunchConfiguration('backend', default='auto')}],
        output='screen'
    )
    lidar = Node(
        package='smart_robot_ai_perception',
        executable='lidar_node',
        name='lidar_node',
        output='screen'
    )
    fusion = Node(
        package='smart_robot_ai_perception',
        executable='fusion_node',
        name='fusion_node',
        parameters=[{'camera_fov_deg': LaunchConfiguration('fov', default='60.0')}],
        output='screen'
    )
    controller = Node(
        package='smart_robot_ai_perception',
        executable='controller_node',
        name='controller_node',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([detector, lidar, fusion, controller])
