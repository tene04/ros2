import pygame
import sys

# Inicialización
pygame.init()
ancho, alto = 800, 600
WIN = pygame.display.set_mode((ancho, alto))
pygame.display.set_caption("Pong 2 Jugadores - Mejor de 3 Puntos")

# Colores y fuente
blanco = (255, 255, 255)
negro = (0, 0, 0)
FONT = pygame.font.SysFont("Arial", 36)
WINNER_FONT = pygame.font.SysFont("Arial", 60)

# Variables de juego
pala_ancho, pala_alto = 10, 100
pelota = 15
pala_velocidad = 6
velocidad_pelota_x, velocidad_pelota_y = 8, 8 

# Inicialización de objetos
left_paddle = pygame.Rect(20, alto//2 - pala_alto//2, pala_ancho, pala_alto)
right_paddle = pygame.Rect(ancho - 30, alto//2 - pala_alto//2, pala_ancho, pala_alto)
ball = pygame.Rect(ancho//2 - pelota//2, alto//2 - pelota//2, pelota, pelota)

# Puntuaciones
score_left = 0
score_right = 0
WIN_SCORE = 3

clock = pygame.time.Clock()

def draw(): # muestra el juego por pantalla
    WIN.fill(negro)
    pygame.draw.rect(WIN, blanco, left_paddle)
    pygame.draw.rect(WIN, blanco, right_paddle)
    pygame.draw.ellipse(WIN, blanco, ball)
    pygame.draw.aaline(WIN, blanco, (ancho // 2, 0), (ancho // 2, alto))

    score_text = FONT.render(f"{score_left}   {score_right}", True, blanco)
    WIN.blit(score_text, (ancho//2 - score_text.get_width()//2, 20))
    pygame.display.flip()

def draw_winner(text): # muestra el ganador por pantalla
    WIN.fill(negro)
    render = WINNER_FONT.render(text, True, blanco)
    WIN.blit(render, (ancho//2 - render.get_width()//2, alto//2 - render.get_height()//2))
    pygame.display.flip()

def handle_movement(keys): # detecta los movimientos de los jugadores
    if keys[pygame.K_w] and left_paddle.top > 0:
        left_paddle.y -= pala_velocidad
    if keys[pygame.K_s] and left_paddle.bottom < alto:
        left_paddle.y += pala_velocidad
    if keys[pygame.K_UP] and right_paddle.top > 0:
        right_paddle.y -= pala_velocidad
    if keys[pygame.K_DOWN] and right_paddle.bottom < alto:
        right_paddle.y += pala_velocidad

def move_ball(): # mueve el balón
    global velocidad_pelota_x, velocidad_pelota_y, score_left, score_right

    ball.x += velocidad_pelota_x
    ball.y += velocidad_pelota_y

    if ball.top <= 0 or ball.bottom >= alto:
        velocidad_pelota_y *= -1

    if ball.colliderect(left_paddle) and velocidad_pelota_x < 0:
        velocidad_pelota_x *= -1
    if ball.colliderect(right_paddle) and velocidad_pelota_x > 0:
        velocidad_pelota_x *= -1

    if ball.left <= 0:
        score_right += 1
        reset_ball()
    if ball.right >= ancho:
        score_left += 1
        reset_ball()

def reset_ball(): # devuelve la pelota al inicio
    global velocidad_pelota_x, velocidad_pelota_y
    ball.center = (ancho // 2, alto // 2)
    velocidad_pelota_x *= -1
    velocidad_pelota_y *= -1
    left_paddle.centery = alto // 2
    right_paddle.centery = alto // 2

# Bucle principal
winner = None
while True:
    clock.tick(60)

    if score_left >= WIN_SCORE:
        winner = "Jugador 1 gana la partida"
    elif score_right >= WIN_SCORE:
        winner = "Jugador 2 gana la partida"

    if winner: # si se termina muestra el ganador
        draw_winner(winner)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        continue

    for event in pygame.event.get(): # cerrar la ventana
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # mostrar el juego
    keys = pygame.key.get_pressed()
    handle_movement(keys)
    move_ball()
    draw()
