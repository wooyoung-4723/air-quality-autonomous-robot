from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'dust_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woo',
    maintainer_email='woo@example.com',
    description='MQTT PM sensor to ROS2 dust heatmap mapper and viewer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_pm_listener = dust_mapping.mqtt_pm_listener:main',
            'dust_mapper = dust_mapping.dust_mapper:main',
            'dust_map_viewer = dust_mapping.dust_map_viewer:main',
        ],
    },
)
