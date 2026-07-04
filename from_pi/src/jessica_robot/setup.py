from glob import glob

from setuptools import find_packages, setup

package_name = 'jessica_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/description', glob('description/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonny',
    maintainer_email='jonathanr4242.utube@gmail.com',
    description='Jessica robot — LLM-driven voice chatbot with ROS 2 hardware control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'jessica_chatbot = jessica_robot.jessica_chatbot:main',
            'hair_led_node   = jessica_robot.hair_led_node:main',
            'pan_tilt_teleop = jessica_robot.pan_tilt_teleop:main',
            'finger_follower = jessica_robot.finger_follower:main',
        ],
    },
)
