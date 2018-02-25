import json
import numpy as np
from obstacleCheck import *

class Map():

    def __init__(self, map_file, traj_file):

        # Parsing the map
        data = json.load(open(map_file))

        self.bounding_polygon = np.array(data["bounding_polygon"])

        #Hitboxes for obstacles for an easy test if a node is outside an obstacle and inside the bounding_polygon
        self.boundingHitBox = [findMaxMin(self.bounding_polygon)]

        # width = maxX - minX, height = maxY - minY
        self.width = 60 #(self.boundingHitBox[0][1] - self.boundingHitBox[0][0]) * 1.5
        self.height = 60 #(self.boundingHitBox[0][2] - self.boundingHitBox[0][3]) * 1.5

        self.formation_positions = np.array(data["formation_positions"])
        self.start_positions = np.array(data["start_positions"])
        self.length = data["vehicle_L"]
        self.a_max = data["vehicle_a_max"]
        self.dt = data["vehicle_dt"]
        self.omega_max = data["vehicle_omega_max"]
        self.phi_max = data["vehicle_phi_max"]
        self.vehicle_t = data["vehicle_t"]
        self.v_max = data["vehicle_v_max"]

        # Parsing the trajectory
        data = json.load(open(traj_file))

        self.traj_t = data["t"]
        self.traj_theta = data["theta"]
        self.traj_x = data["x"]
        self.traj_y = data["y"]

    def isOK(self, point):
        # Check if inside the bounding polygon, inside if
        insidePolygon = isBlocked(point, [self.bounding_polygon], self.boundingHitBox)

        # Check if inside an obstacle
        insideObstacle = isBlocked(point, self.obstacles, self.hitBoxes)

        return insidePolygon and not insideObstacle


