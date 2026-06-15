from setuptools import setup

package_name = 'waypoints_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JITERN',
    maintainer_email='limjitern@gmail.com',
    description='Waypoints publisher for CARLA simulation in ROS',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pub_waypoints_node = waypoints_publisher.pub_waypoints_node:main',
        ],
    },
)
