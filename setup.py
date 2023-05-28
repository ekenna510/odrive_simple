import os
from glob import glob
from setuptools import setup

package_name = 'odrive_simple'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),             
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='ekenna510@verizon.net',
    description='Subscribe to cmd_vel calculate velocities send them to odrive. Use encoder from odrive to publish odom ',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_s=odrive_simple.odrive_simple_node:main'
        ],
    },
)
