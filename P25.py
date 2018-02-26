import numpy as np
import Map
import plotFunctions
import matplotlib.pyplot as plt

# Code for kinematic car

class Node():

    def __init__(self, pos, theta, v):
        self.pos = pos
        self.name = str(self.pos)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        self.theta = theta
        self.x = pos[0]
        self.y = pos[1]
        self.v = v

    def dist(self, otherNode):
        return np.linalg.norm(otherNode.pos - self.pos)

class Formation():

    def __init__(self, formationPositions):
        self.leaderPosition = formationPositions[0]
        self.followerPositions = formationPositions[1:]
        self.distances = np.linalg.norm(self.followerPositions - self.leaderPosition, axis = 1)
        self.angles = np.angle(self.followerPositions[:, 0] - self.leaderPosition[0] + 1j * (self.followerPositions[:, 1] - self.leaderPosition[1])) - np.pi / 2

    def getFollowerPositions(self, currentLeaderPosition, currentLeaderAngle):
        return currentLeaderPosition + np.array([np.multiply(self.distances, np.cos(self.angles + currentLeaderAngle)), np.multiply(self.distances, np.sin(self.angles + currentLeaderAngle))]).T

def formationAcquisition(mapp, formation):
##    mapp.formation_positions = mapp.formation_positions[1:]
    leaderStartPosition = np.array([mapp.traj_x[0], mapp.traj_y[0]])
    leaderStartAngle = mapp.traj_theta[0]
    formationPositions = formation.getFollowerPositions(leaderStartPosition, leaderStartAngle)
    paths = []
##    thetas = np.angle(mapp.formation_positions[:, 0] - mapp.start_positions[:, 0] + 1j * (mapp.formation_positions[:, 1] - mapp.start_positions[:, 1]))
    thetas = np.angle(formationPositions[:, 0] - mapp.start_positions[:, 0] + 1j * (formationPositions[:, 1] - mapp.start_positions[:, 1]))
    TOLERANCE = mapp.v_max * mapp.dt * 8
    DECELERATION_TOLERANCE = mapp.v_max ** 2 / (2 * mapp.a_max)

    i = 0
    for pos in mapp.start_positions:
        tree = []
        startNode = Node(pos, thetas[i], 0)
        formationNode = Node(formationPositions[i], 0, 0)
        tree.append(startNode)
        currentNode = startNode

        # Accelerate with a_max up to v_max and then keep going forward with v_max
        while currentNode.dist(formationNode) > DECELERATION_TOLERANCE:
            print("DECELERATION_TOLERANCE", DECELERATION_TOLERANCE)
            print(currentNode.dist(formationNode))
            if currentNode.v < mapp.v_max:
                a = mapp.a_max
            else:
                a = 0
            newPos = currentNode.pos + (currentNode.v * mapp.dt + (a * mapp.dt ** 2) / 2) * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)])
            newNode = Node(newPos, thetas[i], currentNode.v + a * mapp.dt)
            newNode.parent = currentNode
            currentNode.children.append(newNode)
            tree.append(newNode)
            currentNode = newNode

        # Decelerate so that the vehicle reaches the formation position with v = 0
        a = -currentNode.v / (2 * TOLERANCE)
        while currentNode.dist(formationNode) > TOLERANCE:
            print("TOLERANCE", TOLERANCE)
            print(currentNode.dist(formationNode))
            newPos = currentNode.pos + (currentNode.v * mapp.dt + (a * mapp.dt ** 2) / 2) * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)])
            newNode = Node(newPos, thetas[i], currentNode.v + a * mapp.dt)
            newNode.parent = currentNode
            currentNode.children.append(newNode)
            tree.append(currentNode)
            currentNode = newNode

        path = []
        currentNode = tree[len(tree)-1]
        while not currentNode == startNode:
            ##print(str(currentNode.name))
            path.append(currentNode)
            currentNode = currentNode.parent

        path.append(startNode)
        path.reverse()

        plotFunctions.plotMap(mapp.bounding_polygon)
        plotFunctions.plotTree(tree[0])
        plotFunctions.plotPath(path[len(path)-1])
        
        i += 1

    plt.show()

    paths.append(path)    

##    for tree in trees:

formation = Formation(Map.Map("P25.json", "P25_26_traj.json").formation_positions)    
trees = formationAcquisition(Map.Map("P25.json", "P25_26_traj.json"), formation)
print(formation.getFollowerPositions(np.array([10.1, 15]), np.pi / 2))
