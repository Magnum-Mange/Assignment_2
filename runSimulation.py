import vrep
import numpy as np
import math
##from RRT_KP import findPathKP
##from RRT_DP import findPathDP
##from RRT_DD import findPathDD
##from RRT_KC import findPathKC
import time
from Map import Map
##from commonFunctions import readPath
##from P25 import *
from P26 import *

def writeToFile(obstacles, fileName):
    """The obstacles need to contain the bounding polygon"""
    file = open(fileName, "w")
    numObs = len(obstacles)
    file.write(str(numObs)+"\n")
    for obstacle in obstacles:
        numCorners = len(obstacle)
        file.write(str(numCorners)+"\n")
        for i in range(len(obstacle)):
            point1 = obstacle[i]
            point2 = obstacle[(i+1) % len(obstacle)]
            xy = pointsToPosition(point1, point2)
            file.write(str(xy[0]/10)+"\n")
            file.write(str(xy[1]/10)+"\n")
            file.write(str(pointsToLength(point1, point2)/10)+"\n")
            file.write(str(pointsToAng(point1, point2))+"\n")
    file.close()


def pointsToAng(point1, point2):
    if (point2[0] - point1[0]) != 0:
        gamma = math.atan((point2[1] - point1[1])/(point2[0] - point1[0]))
    else:
        gamma = math.atan(float("inf"))
    return gamma

def pointsToLength(point1, point2):
    length = math.sqrt((point2[0]-point1[0])**2 + (point2[1] - point1[1])**2)
    return length

def pointsToPosition(point1, point2):
    x = (point2[0]-point1[0]) / 2 + point1[0]
    y = (point2[1]-point1[1]) / 2 + point1[1]
    return [x, y]

def velToAng(vel):
    """Takes a velocity and computes the angle between the vector and the x-axis in degrees"""
    radians = math.acos(abs(vel[0])/np.linalg.norm(vel))

    if vel[0] < 0 < vel[1]:
        radians = math.pi - radians
    elif vel[1] < 0 < vel[0]:
        radians = 0 - radians
    elif vel[0] < 0 and vel[1] < 0:
        radians = radians - math.pi
    return radians

def runSimulation():

    # Creates map from json file
    aMap = Map.Map("P25_X.json", "P25_26_traj.json")
    allObstacles = []

    # Writes obstacles to file
    allObstacles.append(aMap.bounding_polygon)
##    for obstacle in aMap.obstacles:
##        allObstacles.append(obstacle)
    writeToFile(allObstacles, "track.txt")

##    rightWall = vrep.simxGetObjectHandle(clientID, "rightWall", vrep.simx_opmode_blocking)
##    topWall = vrep.simxGetObjectHandle(clientID, "topWall", vrep.simx_opmode_blocking)
##    leftWall = vrep.simxGetObjectHandle(clientID, "leftWall", vrep.simx_opmode_blocking)
##    bottomWall = vrep.simxGetObjectHandle(clientID, "bottomWall", vrep.simx_opmode_blocking)
##    vrep.simxSetObjectFloatParameter(clientID, rightWall, 

    # Plans the path
    #path = findPathKP(aMap) # Kinematic Point
    #path = findPathDP(aMap) # Dynamic Point
    #path = findPathDD(aMap) # Differential Drive
##    path = findPathKC(aMap) # Kinematic Car
    formation = Formation(Map.Map("P26_X.json", "P25_26_traj.json").formation_positions)    
    paths = formationAcquisition(Map.Map("P26_X.json", "P25_26_traj.json"), formation)
    acquisitionLength = len(paths[0])
    print(acquisitionLength)
    print(len(paths[1]))
    paths = formationManeuvering(paths, Map.Map("P26_X.json", "P25_26_traj.json"), formation)


    # Reads path from file
    #path = readPath("DP_P3.txt")

    # Initial values
    startNodes = []
    gammas = []
    for path in paths:
        print("hello")
        startNodes.append(path[len(path) - 1])
        if len(path[len(path) - 1].vel) == 0:
            gammas.append(path[len(path) - 1].orientation)
        else:
            gammas.append(velToAng(path[len(path) - 1].vel))
    posZ = 0.025

    # ---------------------- Connects to V-rep
    # Close any open connections
    vrep.simxFinish(-1)
    # Connect to the V-REP continuous server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

    if clientID != -1: # if we connected successfully
        print('Connected to remote API server')

    # --------------------- Setup the simulation

    vrep.simxSynchronous(clientID, True)

    objectNames = []
    numberOfAgents = 10
    objectHandles = [None] * numberOfAgents
    for i in range(numberOfAgents):
        objectNames.append("bubbleRob" + str(i))

        # get the handle for the bubbleRobs
        print(objectHandles[i])
        print(startNodes)
        _, objectHandles[i] = vrep.simxGetObjectHandle(clientID, objectNames[i], vrep.simx_opmode_blocking)

        _ = vrep.simxSetObjectPosition(clientID, objectHandles[i], -1, [startNodes[i].x/10, startNodes[i].y/10, posZ], vrep.simx_opmode_oneshot)
        _ = vrep.simxSetObjectOrientation(clientID, objectHandles[i], -1, [0, 0, gammas[i]],vrep.simx_opmode_oneshot)

    leaderName = "bubbleRob"
    _, leaderHandle = vrep.simxGetObjectHandle(clientID, leaderName, vrep.simx_opmode_blocking)

    _ = vrep.simxSetObjectPosition(clientID, leaderHandle, -1, [aMap.traj_x[0]/10, aMap.traj_y[0]/10, posZ], vrep.simx_opmode_oneshot)
    _ = vrep.simxSetObjectOrientation(clientID, objectHandles[i], -1, [0, 0, aMap.traj_theta[0]],vrep.simx_opmode_oneshot)

    dt = 1
    vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot)

    # --------------------- Start the simulation

    # start our simulation in lockstep with our code
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

    count = 0

    while count < len(paths[0]): # run for 1 simulated second

        if count >= acquisitionLength:
            gamma = aMap.traj_theta[count - acquisitionLength]
            _ = vrep.simxSetObjectPosition(clientID, leaderHandle, -1, [aMap.traj_x[count - acquisitionLength]/10, aMap.traj_y[count - acquisitionLength]/10, posZ], vrep.simx_opmode_oneshot)
            _ = vrep.simxSetObjectOrientation(clientID, leaderHandle, -1, [0, 0, gamma], vrep.simx_opmode_oneshot)
        
        for pathNo in range(len(paths)):
            currentNode = paths[pathNo][count]
            if len(currentNode.vel) == 0:
                gamma = currentNode.orientation
            else:
                gamma = velToAng(currentNode.vel)

            _ = vrep.simxSetObjectPosition(clientID, objectHandles[pathNo], -1, [currentNode.x/10, currentNode.y/10, posZ], vrep.simx_opmode_oneshot)
            _ = vrep.simxSetObjectOrientation(clientID, objectHandles[pathNo], -1, [0, 0, gamma], vrep.simx_opmode_oneshot)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(clientID)
        count += dt

    # Pause simulation
    vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot)
    time.sleep(3)

    # Stop simulation
    vrep.simxStopSimulation(clientID,
            vrep.simx_opmode_blocking)

    # Close the connection to V-REP:
    vrep.simxFinish(clientID)


if __name__ == "__main__":
    runSimulation()
