from glob import glob

from setuptools import setup

package_name = 'custom_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gautham',
    maintainer_email='gauthamanil888@gmail.com',
    description='Teleoperation nodes for the Coco robot arm and wheel base',
    license='Apache-2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'teleop_arm_node = custom_teleop.teleop_arm_node:main',
            'teleop_wheels_node = custom_teleop.teleop_wheels_node:main',
            'cmd_vel_relay = custom_teleop.cmd_vel_relay:main',
        ],
    },
)
