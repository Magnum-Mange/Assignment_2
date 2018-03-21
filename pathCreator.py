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

    def mutation():
        i1 = np.random.randint(len(self.points))
        i2 = np.random.randint(len(self.points))
        while (i2==i1):
            i2 = np.random.randint(len(self.points))
        mem = self.points[i1]
        self.points[i1] = self.points[i2]
        self.points[i2] = mem

    def cross(self,other):
        """
        Crosses with an onther pathObject to create a new one
        """
        cross_length = np.random.randint(len(other.points))
        cross_begin = np.random.randint(len(other.points)-cross_length)
        cross_segment = other.points[cross_begin:cross_begin+cross_length]
        new_points = [-1] * len(self.points)
        new_points[cross_begin:cross_begin+cross_length] = cross_segment
        for i in range(cross_begin):
            for p in self.points:
                if (not p in new_points):
                    new_points[i] = p
        for i in range(cross_begin+cross_length,len(self.points)):
            for p in self.points:
                if (not p in new_points):
                    new_points[i] = p
        ret = PathObject(new_points,0)
        return ret
        



class PathCreator:
    
    def __init__(self,map,type,motion_model="KIN_POINT"):
        self.m = map
        self.g = self.m.visibility_graph()
        self.g.compute_shorts()
        self.type = type
        self_motion = motion_model
        self.gen_mut = 0.2
        self.gen_cross = 0.3

    def random_path(self):
        # In this path, indexes below "num_agents" are the agents, and above are the points of interest.
        total_points = self.m.num_agents + len(self.m.poi)
        set_of_points = np.arange(total_points)
        np.random.shuffle(set_of_points)
        return set_of_points




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
            for i in range(len(paths[agent])):
                if (i==0):
                    costs[agent] += self.g.dists[agent][paths[agent][i]+self.m.num_agents]
                else:
                    costs[agent] += self.g.dists[paths[agent][i-1]+self.m.num_agents][paths[agent][i]+self.m.num_agents]
            if (len(paths[agent]) > 0):
                costs[agent] += self.g.dists[paths[agent][-1]+self.m.num_agents][-1-agent]
        #for agent in range(self.m.num_agents):
        #    traj = [self.m.start_positions[agent]] # traj is a buffer where the trajectory of the current agent is stored. It is initialized witht the start position.
        #    complete_path = []
        #    if (len(paths[agent]) > 0):
        #        complete_path += self.g.quick_path(self.g.starts[agent],paths[agent][0])[:-1]
        #    for i in range (1, len(paths[agent])):
        #        new_segment = self.g.quick_path(paths[agent][i-1],paths[agent][i])
        #        complete_path = complete_path + new_segment[:-1]
        #    last_segment = []
        #    if len(complete_path)==0:
        #        last_segment = self.g.quick_path(self.g.starts[agent],self.g.goals[agent])
        #    else:
        #        last_segment = self.g.quick_path(complete_path[-1],self.g.goals[agent])
        #    complete_path += last_segment[:-1]
        #    complete_path = [self.g.starts[agent]] + complete_path + [self.g.goals[agent]]
        #    for i in range(1,len(complete_path)):
        #        costs[agent] += euclid_distance(self.g.get_node(complete_path[i-1]).position,self.g.get_node(complete_path[i]).position)
        return max(costs)/self.m.v_max

    def create_path(self):
        if (self.type == "random"):
            return self.random_path()
        if (self.type == "genetic"):
            population = []
            for i in range(5000):
                points = self.random_path()
                population.append(PathObject(points,self.evaluate_paths(points)))
            for gen in range(5000):
                population = sorted(population,key=lambda pat:pat.cost)
                print(population[0].cost)
                for i in range(int(len(population)*self.gen_cross)):
                    p1 = np.random.randint(int(len(population)*self.gen_cross))
                    p2 = np.random.randint(int(len(population)*self.gen_cross))
                    population[i+int(len(population)*(1-self.gen_cross))] = population[p1].cross(population[p2])
                    population[i+int(len(population)*(1-self.gen_cross))].cost = self.evaluate_paths(population[i+int(len(population)*(1-self.gen_cross))].points)
                for i in range(len(population)):
                    roll = np.random.uniform(1)
                    if (roll < self.gen_mut):
                        population[i] = population[i].mutation()
                        population[i].cost = self.evaluate_paths(population[i].points)
            population = sorted(population,key=lambda pat:pat.cost)
            return population[0].points

