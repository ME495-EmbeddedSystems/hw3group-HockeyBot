"""
Setup for hockeybot package.

Contains traj_calc, cam_node, main, and simple_move.
"""

from setuptools import setup
import os
from glob import glob
package_name = 'hockeybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaz',
    maintainer_email='AvaZahedi2023@u.northwestern.edu',
    description='Enables the robot to play air hockey provided data from CV.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_calc=hockeybot.traj_calc:traj_calc_entry',
            'cam_node = hockeybot.cam_node:frames_entry',
            'main=hockeybot.main:main_entry',
            'simple_move=moveit_helper.simple_move:simple_move_entry'
        ],
    },
)
