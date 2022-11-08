from setuptools import setup

package_name = 'moveit_helper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avaz',
    maintainer_email='AvaZahedi2023@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['move_action_intercept=moveit_helper.move_action_intercept:interept_entry',
                            'marno_move_group_plan=moveit_helper.marno_move_group_plan:interept_entry', # For Marno Test code
                            'marno_move_group_execute=moveit_helper.marno_move_group_execute:interept_entry',
                            'subscriber_joint_states=moveit_helper.subscriber_joint_states:joint_state_entry',
                            'inverse=moveit_helper.inverse:IK_entry'
        ],
    },
)
