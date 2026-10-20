from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'webots_digits'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.wbt")),
        ),
        (
            os.path.join("share", package_name, "protos"),
            glob(os.path.join("protos", "*.proto")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Ezhov',
    maintainer_email='iamezhov@gmail.com',
    description='ROS 2 package for simulating TurtleBot3 and digits 03 in Webots',
    license='UNLICENSED',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
