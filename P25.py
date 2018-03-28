import numpy as np
import Map
import plotFunctions
import matplotlib.pyplot as plt
import cProfile

class Node():

    def __init__(self, pos, theta, v, vel):
        self.pos = pos
        self.name = str(self.pos)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        self.theta = theta
        self.x = pos[0]
        self.y = pos[1]
        self.v = v
        self.vel = vel

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
    TOLERANCE = mapp.v_max * mapp.dt * 4
    DECELERATION_TOLERANCE = mapp.v_max ** 2 / (2 * mapp.a_max)

    i = 0
    for pos in mapp.start_positions:
        path = []
        startNode = Node(pos, thetas[i], 0, np.array([0, 0]))
        formationNode = Node(formationPositions[i], 0, 0, np.array([0, 0]))
        path.append(startNode)
        currentNode = startNode

        # Accelerate with a_max up to v_max and then keep going forward with v_max
        while currentNode.dist(formationNode) > DECELERATION_TOLERANCE:
##            print("DECELERATION_TOLERANCE", DECELERATION_TOLERANCE)
##            print(currentNode.dist(formationNode))
            if currentNode.v < mapp.v_max:
                a = mapp.a_max
            else:
                a = 0
            newPos = currentNode.pos + (currentNode.v * mapp.dt + (a * mapp.dt ** 2) / 2) * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)])
            newNode = Node(newPos, thetas[i], currentNode.v + a * mapp.dt, np.array([0, 0]))
            newNode.parent = currentNode
            currentNode.children.append(newNode)
            path.append(newNode)
            currentNode = newNode

        # Decelerate so that the vehicle reaches the formation position with v = 0
        a = -currentNode.v / (2 * TOLERANCE)
        while currentNode.dist(formationNode) > TOLERANCE:
##            print("TOLERANCE", TOLERANCE)
##            print(currentNode.dist(formationNode))
            newPos = currentNode.pos + (currentNode.v * mapp.dt + (a * mapp.dt ** 2) / 2) * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)])
            newNode = Node(newPos, thetas[i], currentNode.v + a * mapp.dt, np.array([0, 0]))
            newNode.parent = currentNode
            currentNode.children.append(newNode)
            path.append(currentNode)
            currentNode = newNode

##        path = []
##        currentNode = tree[len(tree)-1]
##        while not currentNode == startNode:
##            ##print(str(currentNode.name))
##            path.append(currentNode)
##            currentNode = currentNode.parent

        plotFunctions.plotMap(mapp.bounding_polygon)
        plotFunctions.plotTree(path[0])
        plotFunctions.plotPath(path[len(path)-1])
        
        paths.append(path)

        i += 1

    plt.show()

    return paths

def pidTuner():
    nodeposs = []
    Kp = 0.006
    Ki = 0.0001
    Kd = 1
    dt = 0.1
    eTot = 0
    oldE = 0
    wantedPoss = np.heaviside(np.linspace(-100, 1000, 1100), 1)
    currentNode = Node(np.array([0, 0]), 0, 0, np.array([0, 0]))
    for t in range(1100):
        e = wantedPoss[t] - currentNode.pos[0]
        a = Kp * e + Ki * eTot + Kd * ((e - oldE) / dt)
        currentNode = Node(currentNode.pos + currentNode.vel[0] * dt + (a * dt ** 2) / 2, 0, 0, currentNode.vel + a * dt)
        nodeposs.append(currentNode.pos[0])
        eTot += e
        oldE = e

    plt.plot(np.linspace(1, 1100, num = 1100), wantedPoss, 'b')
    plt.plot(np.linspace(1, 1100, num = 1100), nodeposs)
    plt.show()

def formationManeuvering(acquisitionPaths, mapp, formation):
    paths = []
    Kp = 0.006
    Ki = 0.0001
    Kd = 1

    leaderStartingSpeed = np.array([mapp.traj_x[1] - mapp.traj_x[0], mapp.traj_y[1] - mapp.traj_y[0]]) / mapp.dt
    
    for pathNo in range(len(acquisitionPaths)):
        print("pathNo: ", pathNo)
        path = acquisitionPaths[pathNo]
        eOld = 0
        eTot = np.array([0.0, 0.0])
        es = np.zeros(len(mapp.traj_t))
        eSum = np.zeros(len(mapp.traj_t))
        path[len(path) - 1].vel = leaderStartingSpeed
        currentNode = path[len(path) - 1]
        startNode = currentNode
        for node in range(0, len(mapp.traj_t) - 1):
            #print("node: ", node)
##            print(currentNode.pos)
##            print(formation.getFollowerPositions(np.array([mapp.traj_x[node], mapp.traj_y[node]]), mapp.traj_theta[node])[pathNo])
            e = formation.getFollowerPositions(np.array([mapp.traj_x[node], mapp.traj_y[node]]), mapp.traj_theta[node])[pathNo] - currentNode.pos
            a = Kp * e + Ki * eTot + Kd * ((e - eOld) / mapp.dt)
            if np.linalg.norm(a) > mapp.a_max:
                a *= np.sqrt(mapp.a_max / np.linalg.norm(a))
##            print("e: ", e)
##            print("a: ", a)
            #print(currentNode.v * mapp.dt * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)]))
            #print(currentNode.pos)
##            newPos = currentNode.pos + currentNode.v * mapp.dt * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)]) + (a * mapp.dt ** 2) / 2
            newPos = currentNode.pos + currentNode.vel * mapp.dt + (a * mapp.dt ** 2) / 2
            #print(newPos)
##            newVel = currentNode.v * np.array([np.cos(currentNode.theta), np.sin(currentNode.theta)]) + a * mapp.dt
            newVel = currentNode.vel + a * mapp.dt
            #print(newVel)
            newTheta = np.angle(np.dot(newVel, np.array([1, 1j])))
##            print(np.degrees(newTheta))
            #print(np.array([1, 1j]))
            #print(np.multiply(newVel, np.array([1, 1j])))
            newNode = Node(newPos, newTheta, np.linalg.norm(newVel), newVel)
            newNode.parent = currentNode
            currentNode.children.append(newNode)
            path.append(newNode)
            eOld = e
            eTot += e
            es[node] = np.linalg.norm(e)
            currentNode = newNode

##        path = []
##        currentNode = tree[len(tree)-1]
##        while not currentNode == startNode:
##            ##print(str(currentNode.name))
##            path.append(currentNode)
##            currentNode = currentNode.parent

##        for node in path:
##            print(node.x, node.y)

##        plotFunctions.plotMap(mapp.bounding_polygon)
##        plotFunctions.plotTree(path[0])
##        plotFunctions.plotPath(path[len(path)-1])
        eSum += es
        paths.append(path)

##        plt.plot(mapp.traj_t, es, 'b')

##    plt.plot(mapp.traj_t, eSum, 'r')

    for path in paths:
        for node in range(len(max(paths, key = len)) - len(path)):
            currentNode = path[len(path) - 1]
            newNode = Node(currentNode.pos, currentNode.theta, currentNode.v, currentNode.vel)
            newNode.parent = currentNode
            currentNode.children.append(newNode)
            path.append(newNode)

    plt.plot(mapp.traj_x, mapp.traj_y, 'b')
    
##    plt.show()        

    return paths

##    for tree in trees:

##formation = Formation(Map.Map("P25.json", "P25_26_traj.json").formation_positions)    
##paths = formationAcquisition(Map.Map("P25.json", "P25_26_traj.json"), formation)
##paths = formationManeuvering(paths, Map.Map("P25.json", "P25_26_traj.json"), formation)

##pidTuner()

##for i in range(15):
##    print(paths[0][i].x, paths[0][i].y)
