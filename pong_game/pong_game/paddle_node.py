#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pong_game.msg import PaddlePosition

import pygame

class PaddleController(Node):
    def __init__(self, player_name):
        super().__init__(f'{player_name}_paddle')
        self.player_name = player_name
        
        # Parameters
        self.declare_parameter('paddle_speed', 5)
        self.paddle_speed = self.get_parameter('paddle_speed').value
        
        # Publisher
        self.publisher = self.create_publisher(
            PaddlePosition,
            f'{player_name}_position',
            10
        )
        
        # Initialize pygame for key input
        pygame.init()
        pygame.display.set_mode((1, 1), pygame.NOFRAME)  # Tiny invisible window
        
        # Game state
        self.y_position = 300  # Initial position
        self.timer = self.create_timer(0.016, self.update)  # ~60Hz
    
    def update(self):
        keys = pygame.key.get_pressed()
        
        # Move paddle based on player
        if self.player_name == 'player1':
            if keys[pygame.K_w]:
                self.y_position -= self.paddle_speed
            if keys[pygame.K_s]:
                self.y_position += self.paddle_speed
        else:  # player2
            if keys[pygame.K_UP]:
                self.y_position -= self.paddle_speed
            if keys[pygame.K_DOWN]:
                self.y_position += self.paddle_speed
        
        # Keep paddle on screen (assuming standard 600 height)
        self.y_position = max(0, min(500, self.y_position))
        
        # Publish position
        msg = PaddlePosition()
        msg.y_position = float(self.y_position)
        msg.player_name = self.player_name
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Determine which player this node controls
    import sys
    if len(sys.argv) < 2:
        print("Usage: paddle_node.py player1|player2")
        return
    
    player_name = sys.argv[1]
    if player_name not in ['player1', 'player2']:
        print("Player must be either 'player1' or 'player2'")
        return
    
    paddle_controller = PaddleController(player_name)
    rclpy.spin(paddle_controller)
    paddle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()