from setuptools import setup
import os
from glob import glob

package_name = 'tof_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonny',
    maintainer_email='jonathanr4242.utube@gmail.com',
    description="ROS 2 package publishing grayscale depth images from the Arducam ToF camera on Jessica's front.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_publisher = tof_publisher.tof_publisher:main',
        ],
    },
)
