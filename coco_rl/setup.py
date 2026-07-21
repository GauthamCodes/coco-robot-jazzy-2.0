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
        ],
    },
)
