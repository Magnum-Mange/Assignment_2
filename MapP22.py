import json
import numpy as np
from obstacleCheck import *
from Graph import Graph, Node
from helpers import *

class NumpyEncoder(json.JSONEncoder):
    """
        Helper class to encode numpy variables in a JSON file.
    """
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class MapP22:
    """
        Object storing the map characteristics for the P22.
    """


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
        """
            Add trajectories to the object
        """
        self.trajectories = trajectories

    def to_JSON(self,file_path):
        """
            Dumps the data into a JSON file
        """
        with open(file_path,'w') as f:
            d = self.__dict__
            json.dump(self.__dict__,f,cls=NumpyEncoder)

    def is_visible(self,n1,n2):
        """
            Checks if n1 is visible from n2.
            n1 and n2 are given as list [x,y]
        """
        line = [n1.position,n2.position]
        for i in range(len(self.bounding_polygon)-1):
            segment = [self.bounding_polygon[i],self.bounding_polygon[i+1]]
            if (intersect(segment,line)):
                return False
        segment = [self.bounding_polygon[len(self.bounding_polygon)-1],self.bounding_polygon[0]]
        if (intersect(segment, line)):
            return False
        for obs in self.obstacles:
            for i in range(len(obs)-1):
                segment = [obs[i],obs[i+1]]
                if (intersect(segment,line)):
                    return False
            segment = [obs[len(obs)-1],obs[0]]
            if (intersect(segment, line)):
                return False
        return True

    def visibility_graph(self):
        """
            Contruct and returns the visibility graph associated with the map.
        """
        graph = Graph(self.num_agents)
        num_of_nodes = 0
        for i in range(self.num_agents):
            n1 = Node(type="START",position_x=self.start_positions[i][0],position_y=self.start_positions[i][1],index = i)
            graph.starts[i] = n1.index
            n2 = Node(type="GOAL",position_x=self.goals[i][0],position_y=self.goals[i][1], index = -1-i)
            graph.goals[i] = n2.index
            graph.add_node(n1)
            graph.add_node(n2)
        for i in range(len(self.poi)):
            n = Node(type="POI",position_x=self.poi[i][0],position_y=self.poi[i][1],index=self.num_agents+i)
            graph.add_node(n)
        for i in range(self.num_obstacles):
            for j in range (len(self.obstacles[i])):
                p1 = [0,0]
                p2 = [0,0]
                p3 = [0,0]
                if (j==0):
                    p1 = self.obstacles[i][-1]
                else:
                    p1 = self.obstacles[i][j-1]
                if (j == len(self.obstacles[i])-1):
                    p3 = self.obstacles[i][0]
                else:
                    p3 = self.obstacles[i][j+1]
                p2 = self.obstacles[i][j]
                offset = [(2*p2[0]-p1[0]-p3[0])*0.01,(2*p2[1]-p1[1]-p3[1])*0.01]
                n = Node(type="CORNER",position_x=p2[0]+offset[0],position_y=p2[1]+offset[1],index=graph.size)
                graph.add_node(n)
        graph.update_visibility(self)
        return graph

