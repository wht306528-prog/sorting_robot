"""Launch the USB camera ping-pong sorting demo."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_camera = LaunchConfiguration('start_camera')
    video_device = LaunchConfiguration('video_device')
    image_topic = LaunchConfiguration('image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    use_depth = LaunchConfiguration('use_depth')
    depth_window_px = LaunchConfiguration('depth_window_px')
    active_tray_id = LaunchConfiguration('active_tray_id')
    process_every_n_frames = LaunchConfiguration('process_every_n_frames')
    f407_host = LaunchConfiguration('f407_host')
    f407_port = LaunchConfiguration('f407_port')

    return LaunchDescription(
        [
            DeclareLaunchArgument('start_camera', default_value='true'),
            DeclareLaunchArgument('video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('image_topic', default_value='/image_raw'),
            DeclareLaunchArgument('depth_image_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
            DeclareLaunchArgument('use_depth', default_value='false'),
            DeclareLaunchArgument('depth_window_px', default_value='5'),
            DeclareLaunchArgument('active_tray_id', default_value='1'),
            DeclareLaunchArgument('process_every_n_frames', default_value='3'),
            DeclareLaunchArgument('f407_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('f407_port', default_value='9000'),
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='pingpong_usb_camera',
                output='screen',
                condition=IfCondition(start_camera),
                parameters=[
                    {
                        'video_device': video_device,
                    }
                ],
            ),
            Node(
                package='sorting_vision',
                executable='pingpong_realtime_node',
                name='pingpong_realtime_node',
                output='screen',
                parameters=[
                    {
                        'color_image_topic': image_topic,
                        'depth_image_topic': depth_image_topic,
                        'use_depth': use_depth,
                        'depth_window_px': depth_window_px,
                        'active_tray_id': active_tray_id,
                        'process_every_n_frames': process_every_n_frames,
                    }
                ],
            ),
            Node(
                package='sorting_driver',
                executable='matrix_tcp_sender',
                name='matrix_tcp_sender',
                output='screen',
                parameters=[
                    {
                        'tray_matrix_topic': '/sorting/tray_matrix',
                        'f407_host': f407_host,
                        'f407_port': f407_port,
                    }
                ],
            ),
        ]
    )
