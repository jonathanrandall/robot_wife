from setuptools import setup
import os
from glob import glob

package_name = 'stereo_pose_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.npz') + glob('config/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonny',
    maintainer_email='jonathanr4242.utube@gmail.com',
    description='Rectifies stereo camera frames and runs MediaPipe pose estimation on each eye.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_pose_node = stereo_pose_publisher.stereo_pose_node:main',
        ],
    },
)
