from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_sdk2_driver',
            executable='livox_mid360_node',
            name='livox_mid360',
            output='screen',
            parameters=[
                {'configFilePath': '/home/drone/ros2_drone_ws/src/livox_sdk2_driver/config/MID360_config.json'},
                {'pointcloud_topic': 'livox/pt_cloud'},
                {'imu_topic': 'livox/imu'},
                {'ptcloud_publish_rate': 10.0},
                {'imu_publish_rate': 50.0},
                {'lidar_translation': [0.15, 0.0, 0.23]},  # z. B. 15 cm nach vorne, 23 cm nach oben
                {'lidar_rotation_rpy_deg': [0.0, 70.0, 0.0]},  # z. B. leicht nach unten geneigt
            ]
        )
    ])