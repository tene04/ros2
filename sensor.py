from robot import *
from game import *

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
        map, objects = self.game.get_info()
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
                point = (x, y)
                        
                if collide:
                    break

                if not map.collidepoint(point):
                    observation.append(dist)
                    collide = True
                    break

                for obj in objects:
                    if obj.collidepoint(point):
                        observation.append(dist)
                        collide = True
                        break
                    
            if not collide: 
                # if there is not obstacle in the range, we will append the max distance to the observation
                observation.append(self.range)

        self.observation = observation
        
        



