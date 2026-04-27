from glob import glob
from setuptools import find_packages, setup

package_name = 'sorting_driver'

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
    description='Driver and communication nodes for the seedling tray sorting robot.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'matrix_protocol_printer = sorting_driver.serial_node:main',
            'matrix_tcp_sender = sorting_driver.tcp_node:main',
        ],
    },
)
