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

from setuptools import find_packages, setup

package_name = 'coco_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # yard_params.yaml is the single source of Yard geometry and is
        # read at runtime by coco_sim.yard, so it has to be installed
        # rather than left in the source tree.
        ('share/' + package_name + '/worlds',
            ['worlds/yard_params.yaml']),
    ],
    # Pinned, not floated. The package exists to be dimensionally and
    # dynamically faithful to Gazebo, and the calibrated contact
    # parameters in coco_sim/mjcf.py were fitted against MuJoCo 3.11.0's
    # solver. A minor-version bump can move contact behaviour, which
    # would silently invalidate the fit and the numbers in RESULTS.md.
    install_requires=['setuptools', 'mujoco==3.11.0'],
    zip_safe=True,
    maintainer='gautham',
    maintainer_email='gauthamanil888@gmail.com',
    description='Generates the MJCF model of the Coco base from coco_config',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'write_mjcf = coco_sim.mjcf:main',
        ],
    },
)
