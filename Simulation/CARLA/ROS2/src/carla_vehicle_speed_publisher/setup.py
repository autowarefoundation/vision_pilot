from setuptools import find_packages, setup

package_name = 'carla_vehicle_speed_publisher'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atanasko Boris Mitrev',
    maintainer_email='atanasko.mitrev@autoware.org',
    description='carla_vehicle_speed_publisher for CARLA simulation in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_vehicle_speed_publisher_node = carla_vehicle_speed_publisher.carla_vehicle_speed_publisher_node:main',
            'velocity_status_bridge_node = carla_vehicle_speed_publisher.velocity_status_bridge_node:main'
        ],
    },
)
