from setuptools import setup

package_name = 'moveit_helper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/simple_move.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaz',
    maintainer_email='AvaZahedi2023@u.northwestern.edu',
    description='Contains simple_move node which allows a user to specify an end-effector location to plan and/or execute a path to.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_move=moveit_helper.simple_move:simple_move_entry'
        ],
    },
)
