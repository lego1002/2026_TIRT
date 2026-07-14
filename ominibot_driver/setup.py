import os
from glob import glob

from setuptools import setup

package_name = 'ominibot_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lego_rpi_4g',
    maintainer_email='lego@anvil.bot',
    description='ROS 2 driver for the OminiBotHV mecanum controller board.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ominibot_driver_node = ominibot_driver.driver_node:main',
        ],
    },
)
