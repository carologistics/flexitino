#!/usr/bin/env python
from setuptools import setup

package_name = 'caro_skills_flexbe_behaviors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grip_sm = caro_skills_flexbe_behaviors.grip_sm',
            'gripper_whole_process_sm = caro_skills_flexbe_behaviors.gripper_whole_process_sm',
            'move_above_target_sm = caro_skills_flexbe_behaviors.move_above_target_sm',
            'move_forward_sm = caro_skills_flexbe_behaviors.move_forward_sm',
            'calibrate_sm = caro_skills_flexbe_behaviors.calibrate_sm',

        ],
    },
)
