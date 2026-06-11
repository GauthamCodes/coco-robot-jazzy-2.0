from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coco_config'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gautham',
    maintainer_email='gauthamanil888@gmail.com',
    description='Configuration package for Coco robot parameters',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_monitor = coco_config.joint_state_monitor:main',
            'diagnostics_node = coco_config.diagnostics_node:main',
        ],
    },
)
