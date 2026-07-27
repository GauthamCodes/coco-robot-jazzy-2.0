# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'teleop_arm_node = custom_teleop.teleop_arm_node:main',
            'teleop_wheels_node = custom_teleop.teleop_wheels_node:main',
            'cmd_vel_relay = custom_teleop.cmd_vel_relay:main',
            'cmd_vel_arbiter = custom_teleop.cmd_vel_arbiter:main',
        ],
    },
)
