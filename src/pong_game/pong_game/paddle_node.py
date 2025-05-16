#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pong_interfaces.msg import PaddlePosition
import keyboard
import sys
import time

class PaddleController(Node):
    def __init__(self, player_name):
        super().__init__(f'{player_name}_paddle')
        self.player_name = player_name
        self.publisher = self.create_publisher(PaddlePosition, f'{player_name}_position', 10)
        self.paddle_speed = 5
        self.y_position = 300
        self.get_logger().info(f"Controlador de {player_name} inicializado (Modo keyboard)")

    def update_position(self):
        moved = False

        if self.player_name == 'player1':
            if keyboard.is_pressed('w'):
                self.y_position -= self.paddle_speed
                moved = True
            elif keyboard.is_pressed('s'):
                self.y_position += self.paddle_speed
                moved = True
        elif self.player_name == 'player2':
            if keyboard.is_pressed('up'):
                self.y_position -= self.paddle_speed
                moved = True
            elif keyboard.is_pressed('down'):
                self.y_position += self.paddle_speed
                moved = True

        if moved:
            self.y_position = max(0, min(600, self.y_position))
            msg = PaddlePosition()
            msg.y_position = float(self.y_position)
            msg.player_name = self.player_name
            self.publisher.publish(msg)
            self.get_logger().info(f"Posición actualizada: {self.y_position}")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Uso: paddle_node.py player1|player2")
        return

    player_name = sys.argv[1]
    if player_name not in ['player1', 'player2']:
        print("El jugador debe ser 'player1' o 'player2'")
        return

    controller = PaddleController(player_name)

    try:
        while rclpy.ok():
            controller.update_position()
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
