"""启动乒乓球分拣演示链路。"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 启动参数这里只保存引用，真正的值由命令行或脚本传入。
    start_camera = LaunchConfiguration('start_camera')
    start_realsense = LaunchConfiguration('start_realsense')
    realsense_color_profile = LaunchConfiguration('realsense_color_profile')
    realsense_depth_profile = LaunchConfiguration('realsense_depth_profile')
    video_device = LaunchConfiguration('video_device')
    image_topic = LaunchConfiguration('image_topic')
    color_camera_info_topic = LaunchConfiguration('color_camera_info_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    use_depth = LaunchConfiguration('use_depth')
    use_undistort = LaunchConfiguration('use_undistort')
    depth_window_px = LaunchConfiguration('depth_window_px')
    expected_tray_count = LaunchConfiguration('expected_tray_count')
    process_every_n_frames = LaunchConfiguration('process_every_n_frames')
    geometry_method = LaunchConfiguration('geometry_method')
    split_wide_large_dark_rects = LaunchConfiguration('split_wide_large_dark_rects')
    large_dark_max_single_width_ratio = LaunchConfiguration('large_dark_max_single_width_ratio')
    relax_split_structure = LaunchConfiguration('relax_split_structure')
    geometry_dark_threshold = LaunchConfiguration('geometry_dark_threshold')
    geometry_projection_active_ratio = LaunchConfiguration('geometry_projection_active_ratio')
    geometry_projection_smooth_px = LaunchConfiguration('geometry_projection_smooth_px')
    geometry_min_width_ratio = LaunchConfiguration('geometry_min_width_ratio')
    geometry_min_height_ratio = LaunchConfiguration('geometry_min_height_ratio')
    geometry_min_area_ratio = LaunchConfiguration('geometry_min_area_ratio')
    geometry_x_padding_px = LaunchConfiguration('geometry_x_padding_px')
    geometry_morphology_kernel_px = LaunchConfiguration('geometry_morphology_kernel_px')
    geometry_edge_roi_padding_px = LaunchConfiguration('geometry_edge_roi_padding_px')
    dark_threshold = LaunchConfiguration('dark_threshold')
    close_kernel_ratio = LaunchConfiguration('close_kernel_ratio')
    min_area_ratio = LaunchConfiguration('min_area_ratio')
    side_band_ratio = LaunchConfiguration('side_band_ratio')
    roi_radius_ratio = LaunchConfiguration('roi_radius_ratio')
    min_white_ratio = LaunchConfiguration('min_white_ratio')
    min_white_component_ratio = LaunchConfiguration('min_white_component_ratio')
    min_white_shape_component_ratio = LaunchConfiguration('min_white_shape_component_ratio')
    min_white_component_diameter_ratio = LaunchConfiguration('min_white_component_diameter_ratio')
    min_white_circularity = LaunchConfiguration('min_white_circularity')
    max_white_center_offset_ratio = LaunchConfiguration('max_white_center_offset_ratio')
    min_ball_ratio = LaunchConfiguration('min_ball_ratio')
    min_yellow_component_ratio = LaunchConfiguration('min_yellow_component_ratio')
    min_color_margin = LaunchConfiguration('min_color_margin')
    start_tcp_sender = LaunchConfiguration('start_tcp_sender')
    f407_host = LaunchConfiguration('f407_host')
    f407_port = LaunchConfiguration('f407_port')
    tcp_connection_log_interval_sec = LaunchConfiguration('tcp_connection_log_interval_sec')
    tcp_send_log_interval_sec = LaunchConfiguration('tcp_send_log_interval_sec')

    return LaunchDescription(
        [
            # 这些参数由 scripts/demo_pingpong.sh 和 scripts/run_pingpong_demo.sh 统一注入。
            DeclareLaunchArgument('start_camera', default_value='true'),
            DeclareLaunchArgument('start_realsense', default_value='false'),
            DeclareLaunchArgument('realsense_color_profile', default_value='640x480x15'),
            DeclareLaunchArgument('realsense_depth_profile', default_value='640x480x15'),
            DeclareLaunchArgument('video_device', default_value='/dev/video0'),
            DeclareLaunchArgument('image_topic', default_value='/image_raw'),
            DeclareLaunchArgument('color_camera_info_topic', default_value='/camera/camera/color/camera_info'),
            DeclareLaunchArgument('depth_image_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
            DeclareLaunchArgument('use_depth', default_value='false'),
            DeclareLaunchArgument('use_undistort', default_value='true'),
            DeclareLaunchArgument('depth_window_px', default_value='5'),
            DeclareLaunchArgument('expected_tray_count', default_value='3'),
            DeclareLaunchArgument('process_every_n_frames', default_value='3'),
            DeclareLaunchArgument('geometry_method', default_value='large_dark_rect'),
            DeclareLaunchArgument('split_wide_large_dark_rects', default_value='true'),
            DeclareLaunchArgument('large_dark_max_single_width_ratio', default_value='0.36'),
            DeclareLaunchArgument('relax_split_structure', default_value='true'),
            DeclareLaunchArgument('geometry_dark_threshold', default_value='85'),
            DeclareLaunchArgument('geometry_projection_active_ratio', default_value='0.10'),
            DeclareLaunchArgument('geometry_projection_smooth_px', default_value='19'),
            DeclareLaunchArgument('geometry_min_width_ratio', default_value='0.08'),
            DeclareLaunchArgument('geometry_min_height_ratio', default_value='0.35'),
            DeclareLaunchArgument('geometry_min_area_ratio', default_value='0.025'),
            DeclareLaunchArgument('geometry_x_padding_px', default_value='10'),
            DeclareLaunchArgument('geometry_morphology_kernel_px', default_value='17'),
            DeclareLaunchArgument('geometry_edge_roi_padding_px', default_value='12'),
            DeclareLaunchArgument('dark_threshold', default_value='95'),
            DeclareLaunchArgument('close_kernel_ratio', default_value='0.045'),
            DeclareLaunchArgument('min_area_ratio', default_value='0.16'),
            DeclareLaunchArgument('side_band_ratio', default_value='0.075'),
            DeclareLaunchArgument('roi_radius_ratio', default_value='0.34'),
            DeclareLaunchArgument('min_white_ratio', default_value='0.36'),
            DeclareLaunchArgument('min_white_component_ratio', default_value='0.30'),
            DeclareLaunchArgument('min_white_shape_component_ratio', default_value='0.18'),
            DeclareLaunchArgument('min_white_component_diameter_ratio', default_value='0.95'),
            DeclareLaunchArgument('min_white_circularity', default_value='0.35'),
            DeclareLaunchArgument('max_white_center_offset_ratio', default_value='0.55'),
            DeclareLaunchArgument('min_ball_ratio', default_value='0.16'),
            DeclareLaunchArgument('min_yellow_component_ratio', default_value='0.12'),
            DeclareLaunchArgument('min_color_margin', default_value='0.035'),
            DeclareLaunchArgument('start_tcp_sender', default_value='true'),
            DeclareLaunchArgument('f407_host', default_value='127.0.0.1'),
            DeclareLaunchArgument('f407_port', default_value='9000'),
            DeclareLaunchArgument('tcp_connection_log_interval_sec', default_value='3.0'),
            DeclareLaunchArgument('tcp_send_log_interval_sec', default_value='2.0'),
            Node(
                # 普通 USB 摄像头入口；topic 模式下 start_camera=false，不启动它。
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
            IncludeLaunchDescription(
                # D435iF/RealSense 入口；启用后由 RealSense 驱动发布 RGB 和对齐深度 topic。
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
                    )
                ),
                condition=IfCondition(start_realsense),
                launch_arguments={
                    'align_depth.enable': 'true',
                    'rgb_camera.color_profile': realsense_color_profile,
                    'depth_module.depth_profile': realsense_depth_profile,
                }.items(),
            ),
            Node(
                # 视觉节点：订阅 RGB/可选深度，发布 debug 图、JSON 和标准 TrayMatrix。
                package='sorting_vision',
                executable='pingpong_realtime_node',
                name='pingpong_realtime_node',
                output='screen',
                parameters=[
                    {
                        'color_image_topic': image_topic,
                        'color_camera_info_topic': color_camera_info_topic,
                        'depth_image_topic': depth_image_topic,
                        'use_depth': use_depth,
                        'use_undistort': use_undistort,
                        'depth_window_px': depth_window_px,
                        'expected_tray_count': expected_tray_count,
                        'process_every_n_frames': process_every_n_frames,
                        'geometry_method': geometry_method,
                        'split_wide_large_dark_rects': split_wide_large_dark_rects,
                        'large_dark_max_single_width_ratio': large_dark_max_single_width_ratio,
                        'relax_split_structure': relax_split_structure,
                        'geometry_dark_threshold': geometry_dark_threshold,
                        'geometry_projection_active_ratio': geometry_projection_active_ratio,
                        'geometry_projection_smooth_px': geometry_projection_smooth_px,
                        'geometry_min_width_ratio': geometry_min_width_ratio,
                        'geometry_min_height_ratio': geometry_min_height_ratio,
                        'geometry_min_area_ratio': geometry_min_area_ratio,
                        'geometry_x_padding_px': geometry_x_padding_px,
                        'geometry_morphology_kernel_px': geometry_morphology_kernel_px,
                        'geometry_edge_roi_padding_px': geometry_edge_roi_padding_px,
                        'dark_threshold': dark_threshold,
                        'close_kernel_ratio': close_kernel_ratio,
                        'min_area_ratio': min_area_ratio,
                        'side_band_ratio': side_band_ratio,
                        'roi_radius_ratio': roi_radius_ratio,
                        'min_white_ratio': min_white_ratio,
                        'min_white_component_ratio': min_white_component_ratio,
                        'min_white_shape_component_ratio': min_white_shape_component_ratio,
                        'min_white_component_diameter_ratio': min_white_component_diameter_ratio,
                        'min_white_circularity': min_white_circularity,
                        'max_white_center_offset_ratio': max_white_center_offset_ratio,
                        'min_ball_ratio': min_ball_ratio,
                        'min_yellow_component_ratio': min_yellow_component_ratio,
                        'min_color_margin': min_color_margin,
                    }
                ],
            ),
            Node(
                # 下游通信节点：把 /sorting/tray_matrix 固定 150 格矩阵发送给 F407/W5500。
                package='sorting_driver',
                executable='matrix_tcp_sender',
                name='matrix_tcp_sender',
                output='screen',
                condition=IfCondition(start_tcp_sender),
                parameters=[
                    {
                        'tray_matrix_topic': '/sorting/tray_matrix',
                        'f407_host': f407_host,
                        'f407_port': f407_port,
                        'connection_log_interval_sec': tcp_connection_log_interval_sec,
                        'send_log_interval_sec': tcp_send_log_interval_sec,
                    }
                ],
            ),
        ]
    )
