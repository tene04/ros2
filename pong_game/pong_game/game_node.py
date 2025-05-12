#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from pong_game.msg import PaddlePosition
from pong_game.srv import Reset

import pygame
import random
import sys
from threading import Thread

class PongGame(Node):
    def __init__(self):
        super().__init__('pong_game')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('paddle_speed', 5),
                ('max_games', 3),
                ('screen_width', 800),
                ('screen_height', 600),
                ('ball_speed_x', 4),
                ('ball_speed_y', 4),
            ]
        )
        
        # Initialize Pygame
        pygame.init()
        self.screen_width = self.get_parameter('screen_width').value
        self.screen_height = self.get_parameter('screen_height').value
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("ROS2 Pong Game")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        
        # Game state
        self.player1_pos = self.screen_height // 2
        self.player2_pos = self.screen_height // 2
        self.ball_pos = [self.screen_width // 2, self.screen_height // 2]
        self.ball_speed = [
            self.get_parameter('ball_speed_x').value,
            self.get_parameter('ball_speed_y').value
        ]
        self.player1_score = 0
        self.player2_score = 0
        self.max_games = self.get_parameter('max_games').value
        self.game_active = True
        
        # ROS2 subscriptions
        self.player1_sub = self.create_subscription(
            PaddlePosition,
            'player1_position',
            self.player1_callback,
            10
        )
        
        self.player2_sub = self.create_subscription(
            PaddlePosition,
            'player2_position',
            self.player2_callback,
            10
        )
        
        # Reset service
        self.reset_service = self.create_service(
            Reset,
            'reset_game',
            self.reset_game_callback
        )
        
        # Start game loop in a separate thread
        self.game_thread = Thread(target=self.game_loop)
        self.game_thread.start()
    
    def player1_callback(self, msg):
        self.player1_pos = int(msg.y_position)
    
    def player2_callback(self, msg):
        self.player2_pos = int(msg.y_position)
    
    def reset_game_callback(self, request, response):
        self.get_logger().info("Resetting game...")
        self.reset_game()
        response.success = True
        return response
    
    def reset_game(self):
        self.ball_pos = [self.screen_width // 2, self.screen_height // 2]
        self.ball_speed = [
            random.choice([-1, 1]) * self.get_parameter('ball_speed_x').value,
            random.choice([-1, 1]) * self.get_parameter('ball_speed_y').value
        ]
        self.player1_score = 0
        self.player2_score = 0
        self.game_active = True
    
    def game_loop(self):
        paddle_width = 15
        paddle_height = 100
        ball_size = 15
        
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            
            # Clear screen
            self.screen.fill((0, 0, 0))
            
            if self.game_active:
                # Move ball
                self.ball_pos[0] += self.ball_speed[0]
                self.ball_pos[1] += self.ball_speed[1]
                
                # Ball collision with top and bottom
                if self.ball_pos[1] <= 0 or self.ball_pos[1] >= self.screen_height - ball_size:
                    self.ball_speed[1] = -self.ball_speed[1]
                
                # Ball collision with paddles
                if (self.ball_pos[0] <= paddle_width and 
                    self.player1_pos <= self.ball_pos[1] <= self.player1_pos + paddle_height):
                    self.ball_speed[0] = -self.ball_speed[0]
                
                if (self.ball_pos[0] >= self.screen_width - paddle_width - ball_size and 
                    self.player2_pos <= self.ball_pos[1] <= self.player2_pos + paddle_height):
                    self.ball_speed[0] = -self.ball_speed[0]
                
                # Ball out of bounds (score)
                if self.ball_pos[0] < 0:
                    self.player2_score += 1
                    self.reset_ball()
                
                if self.ball_pos[0] > self.screen_width:
                    self.player1_score += 1
                    self.reset_ball()
                
                # Check for game over
                if self.player1_score >= self.max_games or self.player2_score >= self.max_games:
                    self.game_active = False
            
            # Draw paddles
            pygame.draw.rect(self.screen, (255, 255, 255), 
                            (0, self.player1_pos, paddle_width, paddle_height))
            pygame.draw.rect(self.screen, (255, 255, 255), 
                            (self.screen_width - paddle_width, self.player2_pos, paddle_width, paddle_height))
            
            # Draw ball
            pygame.draw.rect(self.screen, (255, 255, 255), 
                            (self.ball_pos[0], self.ball_pos[1], ball_size, ball_size))
            
            # Draw scores
            score_text = f"{self.player1_score} - {self.player2_score}"
            score_surface = self.font.render(score_text, True, (255, 255, 255))
            self.screen.blit(score_surface, (self.screen_width // 2 - 30, 10))
            
            # Draw game over message if needed
            if not self.game_active:
                winner = "Player 1" if self.player1_score > self.player2_score else "Player 2"
                game_over_text = f"Game Over! {winner} wins! Press R to reset"
                game_over_surface = self.font.render(game_over_text, True, (255, 255, 255))
                self.screen.blit(game_over_surface, 
                                (self.screen_width // 2 - 180, self.screen_height // 2 - 18))
            
            pygame.display.flip()
            self.clock.tick(60)
    
    def reset_ball(self):
        if self.player1_score < self.max_games and self.player2_score < self.max_games:
            self.ball_pos = [self.screen_width // 2, self.screen_height // 2]
            self.ball_speed = [
                random.choice([-1, 1]) * self.get_parameter('ball_speed_x').value,
                random.choice([-1, 1]) * self.get_parameter('ball_speed_y').value
            ]

def main(args=None):
    rclpy.init(args=args)
    pong_game = PongGame()
    rclpy.spin(pong_game)
    pong_game.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()