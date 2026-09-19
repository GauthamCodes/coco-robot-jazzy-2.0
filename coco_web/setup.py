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

package_name = 'coco_web'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gautham',
    maintainer_email='gauthamanil888@gmail.com',
    description='Versioned WebSocket API and browser control interface '
                'for the Coco robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        # The browser assets. Listed as three explicit globs rather than a
        # recursive walk because data_files is flat: a nested directory
        # silently installs into the parent otherwise, and the vendored
        # joystick would end up beside index.html where the <script src>
        # cannot find it.
        ('share/' + package_name + '/web', glob('web/*.html')),
        ('share/' + package_name + '/web', glob('web/*.css')),
        ('share/' + package_name + '/web', glob('web/*.js')),
        ('share/' + package_name + '/web/vendor', glob('web/vendor/*.js')),
    ],
    entry_points={
        'console_scripts': [
            'platform_server = coco_web.platform_server:main',
        ],
    },
)
