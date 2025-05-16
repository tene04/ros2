from setuptools import setup
import os
from glob import glob

package_name = 'pong_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pong.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'pygame',
    ],

    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Pong game in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'paddle_node = pong_game.paddle_node:main',
            'game_node = pong_game.game_node:main',
        ],
    },
)
