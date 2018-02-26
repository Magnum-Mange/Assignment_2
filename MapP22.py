import json
import numpy as np
from obstacleCheck import *

class MapP22:
    """description of class"""
    def __init__(self, map_file):

        with open(map_file) as f:
            data = json.load(f)
        self.bounding_polygon = np.array(data["bounding_polygon"])

        self.bounding_hitbox = [findMaxMin(self.bounding_polygon)]

        self.width = 60
        self.height = 60

        self.obstacles = []
        self.num_obstacles = 1
        while ("obstacle_"+str(self.num_obstacles) in data.keys()):
            self.obstacles.append(np.array(data["obstacle_"+str(self.num_obstacles)]))
            self.num_obstacles += 1
        self.num_obstacles -= 1
        self.poi = np.array(data["points_of_interest"])
        self.start_positions = np.array(data["start_positions"])
        self.num_agents = self.start_positions.shape[0]
        self.goals = np.array(data["goal_positions"])

        self.length = data["vehicle_L"]
        self.a_max = data["vehicle_a_max"]
        self.dt = data["vehicle_dt"]
        self.omega_max = data["vehicle_omega_max"]
        self.phi_max = data["vehicle_phi_max"]
        self.vehicle_t = data["vehicle_t"]
        self.v_max = data["vehicle_v_max"]




