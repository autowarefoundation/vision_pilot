from setuptools import find_packages, setup

package_name = 'camera_spectator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atanasko',
    maintainer_email='atanasko.mitrev@exo.mk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_spectator_node = camera_spectator.camera_spectator_node:main'
        ],
    },
)
