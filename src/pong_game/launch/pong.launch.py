from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pong_game',
            executable='game_node',
            name='pong_game',
            output='screen',
            parameters=[
                {'paddle_speed': 5},
                {'max_games': 3},
                {'screen_width': 800},
                {'screen_height': 600},
                {'ball_speed_x': 4},
                {'ball_speed_y': 4},
            ],
            arguments=[],
            emulate_tty=True,
            additional_env={'RCL_LOG_LEVEL': 'INFO'}
        ),
        Node(
            package='pong_game',
            executable='paddle_node',
            name='player1_paddle',
            output='screen',
            parameters=[{'paddle_speed': 5}],
            arguments=['player1'],  # Explicit argument
            emulate_tty=True,  # For proper keyboard input
            additional_env={'RCL_LOG_LEVEL': 'INFO'}
        ),
        Node(
            package='pong_game',
            executable='paddle_node',
            name='player2_paddle',
            output='screen',
            parameters=[{'paddle_speed': 5}],
            arguments=['player2'],
            emulate_tty=True,
            additional_env={'RCL_LOG_LEVEL': 'INFO'}
        ),
    ])