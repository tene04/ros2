from setuptools import setup

package_name = 'pong_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/PaddlePosition.msg']),
        ('share/' + package_name + '/srv', ['srv/Reset.srv']),
        ('share/' + package_name + '/launch', ['launch/pong.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='A ROS2 Pong game implementation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_node = pong_game.game_node:main',
            'paddle_node = pong_game.paddle_node:main',
        ],
    },
)