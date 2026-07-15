from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'air_clean_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial', 'paho-mqtt'],
    zip_safe=True,
    maintainer='woo',
    maintainer_email='woo@example.com',
    description='Air quality triggered TurtleBot3 task manager with GUI, Nav2, and custom A* navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_air_quality_node = air_clean_robot.serial_air_quality_node:main',
            'sim_air_quality_publisher = air_clean_robot.sim_air_quality_publisher:main',
            'air_quality_task_manager = air_clean_robot.air_quality_task_manager:main',
            'manual_nav_controller = air_clean_robot.manual_nav_controller:main',
            'air_clean_gui = air_clean_robot.air_clean_gui:main',
            'astar_follower = air_clean_robot.astar_follower:main',
            'air_clean_astar_controller = air_clean_robot.air_clean_astar_controller:main',
            'air_clean_pure_controller = air_clean_robot.air_clean_pure_controller:main',
            'auto_initial_pose = air_clean_robot.auto_initial_pose:main',
            'amcl_pose_mqtt_publisher = air_clean_robot.amcl_pose_mqtt_publisher:main',
            'battery_state_mqtt_publisher = air_clean_robot.battery_state_mqtt_publisher:main',
            'mqtt_cmd_bridge = air_clean_robot.mqtt_cmd_bridge:main',
            'cmd_mqtt_subscriber = air_clean_robot.cmd_mqtt_subscriber:main',
            'dust_cells_mqtt_publisher = air_clean_robot.dust_cells_mqtt_publisher:main',
        ],
    },
)
