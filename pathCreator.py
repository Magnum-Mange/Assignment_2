from MapP22 import MapP22
import numpy as np
from helpers import euclid_distance

class PathCreator:
    
    def __init__(self,map,type,motion_model="KIN_POINT"):
        self.m = map
        self.type = type
        self_motion = motion_model

    def random_path(self):
        total_points = self.m.num_agents + len(self.m.poi)
        set_of_points = np.arange(total_points)
        np.random.shuffle(set_of_points)
        return set_of_points

    def path_to_traj(self,points):
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
            traj = [self.m.start_positions[agent]]
            for inter_point in paths[agent]:
                coords_goals = self.m.poi[inter_point]
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
            print("test")
        return(trajectories)


    def create_path(self):
        if (self.type == "random"):
            return self.random_path()

