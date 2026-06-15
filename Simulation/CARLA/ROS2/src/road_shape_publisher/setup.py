from setuptools import setup

package_name = 'road_shape_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JITERN',
    maintainer_email='limjitern@gmail.com',
    description='Road shape publisher for CARLA simulation in ROS',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'road_shape_publisher_node = road_shape_publisher.road_shape_publisher_node:main',
        ],
    },
)
