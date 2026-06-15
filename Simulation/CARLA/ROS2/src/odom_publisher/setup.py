from setuptools import setup

package_name = 'odom_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JITERN',
    maintainer_email='limjitern@gmail.com',
    description='odom publisher for CARLA simulation in ROS',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'pub_odom_node = odom_publisher.pub_odom_node:main',
        ],
    },
)
