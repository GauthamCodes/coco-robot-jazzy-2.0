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

from setuptools import setup

package_name = 'coco_rl'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The shipped ramp policy, installed so mission.launch.py can
        # default to it. Without a policy on the share path the mission
        # drives to the ramp and stops, and the user has to find a .zip
        # on their own machine before anything runs.
        ('share/' + package_name + '/policies',
         ['policies/phase5_24deg_s0p0.zip']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='gautham',
    maintainer_email='gauthamanil888@gmail.com',
    description='RL ramp traversal for the Coco robot (Gymnasium + SB3 PPO)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'train_ppo = coco_rl.train_ppo:main',
            'evaluate = coco_rl.evaluate:main',
            'plot_curve = coco_rl.plot_curve:main',
            'ramp_driver = coco_rl.ramp_driver:main',
            'terrain_observer = coco_rl.terrain_observer_node:main',
        ],
    },
)
