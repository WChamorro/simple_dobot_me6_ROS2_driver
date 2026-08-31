import os

from glob import glob
from setuptools import find_packages, setup


package_name = 'dobot_e6_joystick'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob(
                'launch/*.launch.py'
            )
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob(
                'config/*.yaml'
            )
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='william',

    maintainer_email='wchamorro@iri.upc.edu',

    description=(
        'ROS 2 joystick controller for the '
        'Dobot Magician E6 robot.'
    ),

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'joystick_node = dobot_e6_joystick.joystick_node:main',
        ],
    },
)