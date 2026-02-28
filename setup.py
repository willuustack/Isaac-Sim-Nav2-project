from setuptools import setup
import os
from glob import glob

package_name = 'isaac_sim_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'tests'), glob('tests/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 Nav2 integration with NVIDIA Isaac Sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_goal_nav = isaac_sim_nav2.multi_goal_navigator:main',
            'bag_recorder = isaac_sim_nav2.bag_recorder:main',
        ],
    },
)
