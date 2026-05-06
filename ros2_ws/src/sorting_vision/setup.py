from glob import glob
from setuptools import find_packages, setup

package_name = 'sorting_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HaiTao Wang',
    maintainer_email='wht306528@gmail.com',
    description='苗盘分拣机器人视觉节点。',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_reference_debug = sorting_vision.debug_tools.aruco_reference_debug:main',
            'camera_info_probe = sorting_vision.probes.camera_info_probe:main',
            'camera_input_probe = sorting_vision.probes.camera_input_probe:main',
            'capture_rgbd_sample = sorting_vision.capture.capture_rgbd_sample:main',
            'depth_point_probe = sorting_vision.probes.depth_point_probe:main',
            'grid_debug_publisher = sorting_vision.nodes.grid_debug_publisher:main',
            'mock_matrix_publisher = sorting_vision.nodes.mock_matrix_publisher:main',
            'offline_tray_debug = sorting_vision.debug_tools.legacy.offline_tray_debug:main',
            'pingpong_realtime_node = sorting_vision.nodes.pingpong_realtime_node:main',
            'real_matrix_publisher = sorting_vision.nodes.real_matrix_publisher:main',
            'tray_rectify_debug = sorting_vision.debug_tools.tray_rectify_debug:main',
            'tray_geometry_debug = sorting_vision.debug_tools.tray_geometry_debug:main',
            'tray_grid_debug = sorting_vision.debug_tools.legacy.tray_grid_debug:main',
            'yolo_object_debug = sorting_vision.debug_tools.yolo_object_debug:main',
        ],
    },
)
