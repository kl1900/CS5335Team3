import os
from glob import glob
from setuptools import setup

package_name = 'patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kuo Lu',
    maintainer_email='kuolu@pm.me',
    description='Patrol node for TurtleBot4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_patrol = patrol.patrol_with_battery:main',
            'patrol_loop = patrol.patrol_loop:main',
        ],
    },
)
