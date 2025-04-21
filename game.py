import pygame
import random
import numpy as np
import time as t

class Sensor:
    def __init__(self, name, robot, game, range):
        self.name = name # sensor name
        self.robot = robot # reference to the robot object
        self.game = game # reference to the game object
        self.range = range # sensor range
        self.observation = None # sensor observation

    def get_info(self):
        # it will be the topic in ros2
        return self.observation
    
    def run(self):
        # the sensor will be activated and will calculate the distance to the obstacles
        map_width, map_height, reward = self.game.get_info()
        position = self.robot.position
        x0, y0 = position

        directions = [
            ( 0, -1), ( 1, -1), ( 1,  0), ( 1,  1),
            ( 0,  1), (-1,  1), (-1,  0), (-1, -1)
        ]
                
        observation = []

        for dx, dy in directions:
            # for each direction, we will check if there is a collision with the map or the objects
            # we only will save the nearest distance to an obstacle
            collide = False
            for dist in range(self.robot.speed, self.range + 1):
                x = x0 + dx * dist
                y = y0 + dy * dist
                point = [x, y]
                        
                if collide:
                    break

                if not (0 < x < map_width and 0 < y < map_height):
                    observation.append(dist)
                    collide = True
                    break

                for obj in self.game.obstacles:
                    if obj.collidepoint(point):
                        observation.append(dist)
                        collide = True
                        break
                    
            if not collide: 
                # if there is not obstacle in the range, we will append the max distance to the observation
                observation.append(self.range)

        observation.append(np.sqrt((x - reward.x)**2 + (y - reward.y**2)))

        self.observation = observation
        
# ------------------------------------------------------------------------------------

class Robot:
    def __init__(self, name, mode, position, game, speed=2, size=20, terminated=False, action=None):
        self.name = name # robot name
        self.position = position  # (x, y) coordinates
        self.size = size # (width, height) of the robot
        self.speed = speed # speed of the robot
        self.sensor = None # reference to the sensor object
        self.game = game # reference to the game object
        self.terminated = terminated # boolean to check if the robot is terminated or not
        self.mode = mode # posible modes [manual', 'automatic']
        pygame.init()
        pygame.display.set_caption('Robot Movement with Obstacles')

    def set_sensor(self, sensor):
        self.sensor = sensor
        
    def check_status(self):
        map_width, map_height, reward = self.game.get_info()
        if any(obj.collidepoint(self.position) for obj in self.game.obstacles) or not (0 < self.position[0] < map_width and 0 < self.position[1] < map_height):
            self.terminated = True
        return 10 if reward.collidepoint(self.position) else 0

    def env(self):
        self.terminated = False
        screen = pygame.display.set_mode(self.game.size)
        self.game.create_obstacles()
        _, _, reward_rect = self.game.get_info()
        clock = pygame.time.Clock()
        starte_positions = [[25, 25], [25, self.game.size[1]-25], [self.game.size[0]-25, 25], [self.game.size[0]-25, self.game.size[1]-25]]
        self.position = random.choice(starte_positions)
        
        return self.terminated, screen, clock, reward_rect
        
    def train(self, action, screen, clock, reward_rect, time):
        change = False
        self.sensor.run()
        observation = self.sensor.get_info()
        reward = self.check_status()
        if not self.terminated:
            if action == 1:
                self.position[1] += self.speed  # Move down
            elif action == 2:
                self.position[0] -= self.speed  # Move right
            elif action == 3:
                self.position[0] += self.speed # Move left
            else:
                self.position[1] -= self.speed # Move up
                
            robot_rect = pygame.Rect(robot.position[0], robot.position[1], robot.size, robot.size)

    
            if time > 2:
                self.game.create_obstacles()
                change = True
                
            clock.tick(60)
        else:
            pygame.quit()

        return observation, self.terminated, reward, change
        
    def run(self):
        self.terminated = self.check_status()
        while not self.terminated:
            self.terminated = self.check_status()
            if self.mode == 'automatic':
                self.sensor.run()
                observation = self.sensor.get_info()
            else:
                self.action = None
                return 'Manual mode activated'
            
# ---------------------------------------------------------------------------------



class Game:
    def __init__(self, size, n_obstacles):
        self.size = size # size of the game window
        self.n_obstacles = n_obstacles # number of obstacles
        self.obstacles = None
    
    def get_info(self):
        return self.size[0], self.size[1], self.create_reward()
    
    def create_reward(self):
        x = random.randint(50, self.size[0] - 50)
        y = random.randint(50, self.size[1] - 50)
        w = 10
        h = 10
        return pygame.Rect(x, y, w, h)
         
    def create_obstacles(self):
        reward = self.create_reward()
        obstacles = []
        for _ in range(self.n_obstacles):
            x = random.randint(50, self.size[0] - 50)
            y = random.randint(50, self.size[1] - 50)
            while x == reward.x and y == reward.y:
                x = random.randint(50, self.size[0] - 50)
                y = random.randint(50, self.size[1] - 50)
            w = random.choice(np.arange(0, 120, 20))
            h = random.choice(np.arange(0, 120, 20))
            obstacles.append(pygame.Rect(x, y, w, h))
        self.obstacles = obstacles

    def start_robot(self):
        starte_positions = [[25, 25], [25, self.size[1]-25], [self.size[0]-25, 25], [self.size[0]-25, self.size[1]-25]]
        robot = Robot('robot', 'automatic', random.choice(starte_positions), self, 5)
        sensor = Sensor('sensor', robot, self, 100) 
        robot.set_sensor(sensor)
        return robot, starte_positions

    def run(self):
        screen = pygame.display.set_mode((self.size[0], self.size[1]))
        _, _, obstacles = self.get_info()
        clock = pygame.time.Clock()
        pygame.init()
        pygame.display.set_caption('Robot Movement with Obstacles')
        robot, started_position = self.start_robot()

        a = t.time()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or pygame.key.get_pressed()[pygame.K_ESCAPE]:
                    running = False

            if robot.mode == 'automatic':
                action, terminated = robot.train()
                print(action, terminated)
            else:
                keys = pygame.key.get_pressed()
                if keys[pygame.K_LEFT]:
                    robot.position[0] -= robot.speed
                elif keys[pygame.K_RIGHT]:
                    robot.position[0] += robot.speed
                elif keys[pygame.K_UP]:
                    robot.position[1] -= robot.speed
                elif keys[pygame.K_DOWN]:
                    robot.position[1] += robot.speed

                robot.position[0] = max(0, min(robot.position[0], self.size[0] - robot.size))
                robot.position[1] = max(0, min(robot.position[1], self.size[1] - robot.size))

                robot_rect = pygame.Rect(robot.position[0], robot.position[1], robot.size, robot.size)
                for obs in obstacles:
                    if robot_rect.colliderect(obs):
                        robot.position = random.choice(started_position)
                        break

            # Draw background and obstacles
            screen.fill((0, 0, 0))  # Black background
            pygame.draw.rect(screen, (0, 255, 0), robot_rect)  # Green robot
            for obs in obstacles:
                pygame.draw.rect(screen, (255, 0, 0), obs)  # Red obstacles
            
            pygame.display.flip()
            b = t.time()
            if b - a > 5:
                a = b
                obstacles = self.create_obstacles()
            
            clock.tick(60)
        
        pygame.quit()


def fitness(network, robot, i, episodes=1,):
    # se evalúa el MLP en el entorno y se saca la media de los episodios
    total_reward = 0.0
    for _ in range(episodes):
        terminated, screen, clock, reward_rect = robot.env()
        done = False
        obs = [sensor.range] * 9
        a = t.time()
        while not done:
            action = np.argmax(network.forward(obs))
            b = t.time()
            obs, terminated, reward, change = robot.train(action, screen, clock, reward_rect, time = b-a)
            total_reward -= 1e-4
            total_reward += reward
            done = terminated
            if change:
                a = b
    return total_reward / episodes

from MLP import *

class neuroevolution():
    # Hemos modificado el algoritmo genético dado en clase para que sea más eficiente y se adopte mejor a los requisitos del gym.
    # Tras probar varias variantes, hemos llegado a la conclusión que funciones como crossover y torneo eran más perjudiciales que 
    # beneficiosas, el primero porque no se adaptaba bien a la estructura de la red y hacía cambios muy bruscos y el segundo porque
    #  en este caso es mejor seleccionar a los mejores individuos y no hacer un torneo entre ellos. Por esta razón hemos partido de 
    # los mejores individuos de la generación anterior para crear la siguiente.

    def __init__(self, input, hidden, output, fitness_func, elite_fraction, pmut, N=100):
        self.input = input # tamaño de la entrada
        self.hidden = hidden # tamaño de la capa oculta, por simplicidad solo hay una
        self.output = output # tamaño de la salida
        self.fitness_func = fitness_func # función de fitness
        self.elite_fraction = elite_fraction # porcentaje de élite que se usará para la reproducción	
        self.pmut = pmut # probabilidad de mutación
        self.N = N # tamaño de la población
        self.poblation = self.create() # población inicial

    def create(self):
        # crea una población de N individuos
        poblation = []
        for _ in range(self.N):
            net = MLP(self.input, self.hidden, self.output)
            poblation.append(net)
        return poblation
    
    def mutate(self, weights):
        # muta los pesos de un individuo
        new_weights = weights.copy()
        for i in range(len(new_weights)):
            if np.random.rand() < self.pmut:
                new_weights[i] += np.random.normal(0, abs(new_weights[i]) * 0.5 + 1e-3) # se altera según el valor del peso
        return new_weights
    
    def evolution(self, ngen, trace, env):
        
        for generation in range(ngen): # para cada generación
            # se evalúa la población y se queda con los mejores individuos
            fitness = [self.fitness_func(net, env, i) for i, net in enumerate(self.poblation)]
            print('---------------------------fin--------------------------')
            elite_count = int(self.elite_fraction * self.N)
            elite_indices = np.argsort(fitness)[-elite_count:]
            elites = [self.poblation[i] for i in elite_indices]

            if generation % trace == 0:
                print(f"Generation {generation} | Max fitness: {max(fitness)} | Avg fitness: {np.mean(fitness)}")

            # con los mejores individuos se crea una nueva población
            new_pobla = []
            for _ in range(self.N):
                parent = np.random.choice(elites)
                child = MLP(self.input, self.hidden, self.output)
                child.set_weights(self.mutate(parent.get_weights()))
                new_pobla.append(child)

            self.poblation = new_pobla

        best_idx = np.argmax([self.fitness_func(net, env, 5) for net in self.poblation])
        best_network = self.poblation[best_idx]
        return best_network # se devuelve el mejor individuo de la última generación 

game = Game((500, 400), 5)
robot = Robot('robot', 'automatic', (0, 0), game, 4)
sensor = Sensor('sensor', robot, game, 100) 
robot.set_sensor(sensor)
neu = neuroevolution(9, [16], 4, fitness, 0.2, 0.1, 100)
best = neu.evolution(1000, 10, robot)
print(best)
