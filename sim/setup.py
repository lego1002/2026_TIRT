import os
from glob import glob

from setuptools import setup

package_name = 'tirt_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lego',
    maintainer_email='gdteyuj123@gmail.com',
    description='2026 TIRT 迷宮機器人 2D 假物理模擬環境',
    license='BSD',
    entry_points={
        'console_scripts': [
            'fake_base = tirt_sim.fake_base_node:main',
            'fake_lidar = tirt_sim.fake_lidar_node:main',
            'make_map = tirt_sim.make_map:main',
        ],
    },
)
