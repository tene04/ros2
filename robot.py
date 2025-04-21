from MLP import *
from sensor import *

class Robot:
    def __init__(self, name, mode, position, game, speed=2, size=20, terminated=False, action=None):
        self.name = name # robot name
        self.position = position  # (x, y) coordinates
        self.size = size # (width, height) of the robot
        self.speed = speed # speed of the robot
        self.sensor = None # reference to the sensor object
        self.game = game # reference to the game object
        self.terminated = terminated # boolean to check if the robot is terminated or not
        self.action = action # action to be taken by the robot
        self.mode = mode # posible modes [manual', 'automatic']

    def set_sensor(self, sensor):
        self.sensor = sensor
        
    def check_Status(self):
        map, objects = self.game.get_info()
        if any(obj.collidepoint(self.position) for obj in objects) or not map.collidepoint(self.position):
            self.terminated = True

    def train(self, action):
        self.sensor.run()
        observation = self.sensor.get_info()

        if action is not None:
            self.action = action
            if self.action == 1:
                self.position[1] += self.speed  # Move up
            elif self.action == 2:
                self.position[0] -= self.speed  # Move left
            elif self.action == 3:
                self.position[0] += self.speed # Move right
            else:
                self.position[1] -= self.speed # Move down
        
        self.terminated = self.check_Status()
        return observation, self.terminated
        
        
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
            
            
    
    

    
