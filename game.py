import pygame
import sys

# Inicializar Pygame
pygame.init()

# Configurar la ventana
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Pelota Rebotando con Obstáculos")

# Colores
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Configurar la pelota
ball_radius = 20
ball_x = width // 2
ball_y = height // 2
ball_speed_x = 5
ball_speed_y = 4

# Obstáculos (Rectángulos)
obstacles = [
    pygame.Rect(300, 200, 200, 20),
    pygame.Rect(150, 400, 500, 20)
]

# Reloj para controlar FPS
clock = pygame.time.Clock()

# Bucle principal
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Mover la pelota
    ball_x += ball_speed_x
    ball_y += ball_speed_y

    # Detectar colisiones con los bordes
    

    # Detectar colisiones con obstáculos
    ball_rect = pygame.Rect(ball_x - ball_radius, ball_y - ball_radius, ball_radius * 2, ball_radius * 2)
    for obstacle in obstacles:
        if ball_rect.colliderect(obstacle):
            # Decide cómo rebotar según el lado del choque
            if abs(obstacle.top - ball_rect.bottom) < 10 and ball_speed_y > 0:
                ball_speed_y *= -1
            elif abs(obstacle.bottom - ball_rect.top) < 10 and ball_speed_y < 0:
                ball_speed_y *= -1
            elif abs(obstacle.left - ball_rect.right) < 10 and ball_speed_x > 0:
                ball_speed_x *= -1
            elif abs(obstacle.right - ball_rect.left) < 10 and ball_speed_x < 0:
                ball_speed_x *= -1

    # Dibujar todo
    screen.fill(WHITE)
    for obstacle in obstacles:
        pygame.draw.rect(screen, BLUE, obstacle)
    pygame.draw.circle(screen, RED, (ball_x, ball_y), ball_radius)
    pygame.display.flip()

    # Limitar los FPS
    clock.tick(60)
