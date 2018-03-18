from MapP22 import MapP22
import numpy as np
from helpers import euclid_distance
from copy import copy
from operator import attrgetter
from builtins import sorted

class PathObject:
    """
        Contains a path and its cost
    """

    def __init__(self, points, cost):
        self.points = points
        self.cost = cost


class PathCreator:
    
    def __init__(self,map,type,motion_model="KIN_POINT"):
        self.m = map
        self.g = self.m.visibility_graph()
        self.g.compute_shorts()
        self.type = type
        self_motion = motion_model

    def random_path(self):
        # In this path, indexes below "num_agents" are the agents, and above are the points of interest.
        total_points = self.m.num_agents + len(self.m.poi)
        set_of_points = np.arange(total_points)
        np.random.shuffle(set_of_points)
        return set_of_points

    #def random_path(self):
    #    agents = np.arange(self.m.num_agents)
    #    np.random.shuffle(agents) #Lit of agents to consider
    #    objectives = copy(self.g.poi)
    #    for i in range(len(agents)):
    #        agent = agents[i]
    #        path = [self.g.get_node(self.g.starts[agent])]
    #        if (i != len(agents)-1):
    #            current_node = path[-1]
    #            while (current_node.type != "GOAL"):
    #                possible_successors = []
    #                if (len(path) > len(self.g.poi)+self.m.num_obstacles*4): #if the path is long, we try to reach the goal
    #                    if (node == self.g.goals[agent]):
    #                        possible_successors = [self.g.goals[agent]]
    #                else:
    #                    # else we prioritize the objectives.
    #                    possible_successors = []
    #                    for node in current_node.neighbors:
    #                        if (node in objectives or node == self.g.goals[agent]):
    #                            possible_successors.append(node)



    def path_to_traj(self,points):
        """
            Converts paths (as list of points) to actual trajectories (list of successive coordinates).
            points is a list 
        """
        paths = [None]*self.m.num_agents
        current_path = []
        current_agent = -1
        for i in points:
            if (i < self.m.num_agents):
                if (current_agent > -1):
                    paths[current_agent] = current_path
                current_agent = i
                current_path = []
            else:
                current_path.append(i-self.m.num_agents)
        for i in points:
            if (i < self.m.num_agents):
                paths[current_agent] = current_path
                break
            else:
                current_path.append(i-self.m.num_agents)
        trajectories = [None]*self.m.num_agents
        for agent in range(self.m.num_agents):
            traj = [self.m.start_positions[agent]] # traj is a buffer where the trajectory of the current agent is stored. It is initialized witht the start position.
            # Creation of the complete path for the agent : since some points are not visible from each other, we have to add new inter points in between.
            complete_path = []
            if (len(paths[agent]) > 0):
                complete_path += self.g.quick_path(self.g.starts[agent],paths[agent][0])
            for i in range (1, len(paths[agent])):
                new_segment = self.g.quick_path(paths[agent][i-1],paths[agent][i])
                complete_path = complete_path + new_segment
            last_segment = []
            if len(complete_path)==0:
                last_segment = self.g.quick_path(self.g.starts[agent],self.g.goals[agent])
            else:
                last_segment = self.g.quick_path(complete_path[-1],self.g.goals[agent])
            complete_path += last_segment
            for inter_point in complete_path: # inter_point is the next point in the path.
                coords_goals = self.g.get_node(inter_point).position
                while (euclid_distance(traj[-1],coords_goals) > self.m.dt * self.m.v_max):
                    new_point = [0,0]
                    new_point[0] = traj[-1][0] + ((coords_goals[0]-traj[-1][0])*self.m.v_max*self.m.dt / euclid_distance(traj[-1],coords_goals))
                    new_point[1] = traj[-1][1] + ((coords_goals[1]-traj[-1][1])*self.m.v_max*self.m.dt / euclid_distance(traj[-1],coords_goals))
                    traj.append(new_point)
                traj.append(coords_goals)
            coords_goals = self.m.goals[agent]
            while (euclid_distance(traj[-1],coords_goals) > self.m.dt * self.m.v_max):
                    new_point = np.zeros((2))
                    new_point[0] = traj[-1][0] + ((coords_goals[0]-traj[-1][0])*self.m.v_max*self.m.dt / euclid_distance(traj[-1],coords_goals))
                    new_point[1] = traj[-1][1] + ((coords_goals[1]-traj[-1][1])*self.m.v_max*self.m.dt / euclid_distance(traj[-1],coords_goals))
                    traj.append(new_point)
            traj.append(coords_goals)
            trajectories[agent] = traj
            #print("test")
        return(trajectories)

    def evaluate_paths(self,points):
        """
            Computes and return the cost of a path given in the compact form
        """
        paths = [None]*self.m.num_agents
        current_path = []
        current_agent = -1
        for i in points:
            if (i < self.m.num_agents):
                if (current_agent > -1):
                    paths[current_agent] = current_path
                current_agent = i
                current_path = []
            else:
                current_path.append(i-self.m.num_agents)
        for i in points:
            if (i < self.m.num_agents):
                paths[current_agent] = current_path
                break
            else:
                current_path.append(i-self.m.num_agents)
        costs = [0] * self.m.num_agents
        for agent in range(self.m.num_agents):
            traj = [self.m.start_positions[agent]] # traj is a buffer where the trajectory of the current agent is stored. It is initialized witht the start position.
            complete_path = []
            if (len(paths[agent]) > 0):
                complete_path += self.g.quick_path(self.g.starts[agent],paths[agent][0])[:-1]
            for i in range (1, len(paths[agent])):
                new_segment = self.g.quick_path(paths[agent][i-1],paths[agent][i])
                complete_path = complete_path + new_segment[:-1]
            last_segment = []
            if len(complete_path)==0:
                last_segment = self.g.quick_path(self.g.starts[agent],self.g.goals[agent])
            else:
                last_segment = self.g.quick_path(complete_path[-1],self.g.goals[agent])
            complete_path += last_segment[:-1]
            complete_path = [self.g.starts[agent]] + complete_path + [self.g.goals[agent]]
            for i in range(1,len(complete_path)):
                costs[agent] += euclid_distance(self.g.get_node(complete_path[i-1]).position,self.g.get_node(complete_path[i]).position)
        return max(costs)/self.m.v_max

    def create_path(self):
        if (self.type == "random"):
            return self.random_path()
        if (self.type == "genetic"):
            population = []
            for i in range(10000):
                points = self.random_path()
                population.append(PathObject(points,self.evaluate_paths(points)))
            population = sorted(population,key=lambda pat:pat.cost)
        return population[0].points

