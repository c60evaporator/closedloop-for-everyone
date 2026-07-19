"""Launch RViz2 preconfigured for the ROS2 data agent topics.

Starts, on the host (or any machine on the same DDS domain as the container):
  - one image_transport republish node per camera, decoding
    /shasou/<cam>/image_raw/compressed -> /shasou/<cam>/image_raw so RViz's
    plain Image displays work without transport support
  - rviz2 with tools/nuscenes/nuscenes_topics.rviz (fixed frame "map", orbit
    view following base_link, LiDAR/Odometry/Path/Detection3DArray/camera
    displays preconfigured)
All nodes run with use_sim_time so stamps follow the /clock published by the
agent.

Host prerequisites (ROS 2 Humble):
    sudo apt install ros-humble-desktop \\
        ros-humble-image-transport-plugins \\
        ros-humble-vision-msgs-rviz-plugins

Usage (ROS_DOMAIN_ID must match the container, default 0):
    ros2 launch tools/nuscenes/rviz_nuscenes.launch.py
"""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

# Must match the TOPIC_NAMESPACE and camera ids of the running agent
# (generalized_ros2_data_agent.py / ros2_data_agent_nuscenes.py).
TOPIC_NAMESPACE = '/shasou'
CAMERA_IDS = [
    'cam_front', 'cam_front_left', 'cam_front_right',
    'cam_back', 'cam_back_left', 'cam_back_right',
]

RVIZ_CONFIG = str(Path(__file__).resolve().parent / 'nuscenes_topics.rviz')


def generate_launch_description():
    use_sim_time = {'use_sim_time': True}

    republishers = [
        Node(
            package='image_transport',
            executable='republish',
            name=f'republish_{camera_id}',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', f'{TOPIC_NAMESPACE}/{camera_id}/image_raw/compressed'),
                ('out', f'{TOPIC_NAMESPACE}/{camera_id}/image_raw'),
            ],
            parameters=[use_sim_time],
        )
        for camera_id in CAMERA_IDS
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', RVIZ_CONFIG],
        parameters=[use_sim_time],
    )

    return LaunchDescription(republishers + [rviz])
