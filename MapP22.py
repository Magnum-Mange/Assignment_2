import json
import numpy as np
from obstacleCheck import *

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class MapP22:
    """description of class"""
    def __init__(self, map_file):

        with open(map_file) as f:
            data = json.load(f)
        self.bounding_polygon = data["bounding_polygon"]

        self.bounding_hitbox = [findMaxMin(self.bounding_polygon)]

        self.width = 60
        self.height = 60

        self.obstacles = []
        self.num_obstacles = 1
        while ("obstacle_"+str(self.num_obstacles) in data.keys()):
            self.obstacles.append(data["obstacle_"+str(self.num_obstacles)])
            self.num_obstacles += 1
        self.num_obstacles -= 1
        self.poi = data["points_of_interest"]
        self.start_positions = data["start_positions"]
        self.num_agents = len(self.start_positions)
        self.goals = data["goal_positions"]

        self.length = data["vehicle_L"]
        self.a_max = data["vehicle_a_max"]
        self.dt = data["vehicle_dt"]
        self.omega_max = data["vehicle_omega_max"]
        self.phi_max = data["vehicle_phi_max"]
        self.vehicle_t = data["vehicle_t"]
        self.v_max = data["vehicle_v_max"]

    def add_traj(self,trajectories):
        self.trajectories = trajectories

    def to_JSON(self,file_path):
        with open(file_path,'w') as f:
            d = self.__dict__
            json.dump(self.__dict__,f,cls=NumpyEncoder)

