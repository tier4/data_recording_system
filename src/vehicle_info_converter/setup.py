import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vehicle_info_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akihiko Shimizu',
    maintainer_email='v-shimaki2507@tier4.jp',
    description='ROS2 node to convert CAN messages to VelocityReport.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_info_converter_node = vehicle_info_converter.vehicle_info_converter_node:main',
        ],
    },
)
