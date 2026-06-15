from setuptools import setup

package_name = 'carla_control_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JITERN',
    maintainer_email='limjitern@gmail.com',
    description='carla_control_publisher for CARLA simulation in ROS',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pub_carla_control = carla_control_publisher.pub_carla_control_node:main',
        ],
    },
)
