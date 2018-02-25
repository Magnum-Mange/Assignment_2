# Hitta Xmin, Xmax, Ymin, Ymax, för alla hinder. Om en punkt har en koordinat större än max eller mindre en min, är den inte i punkten.
# lägg till hitbox till hindret, räkna ut det när hindrena läses in eller nåt
# Ray casting for andra punkter


def makeBoxes(obstacles):
    maxMinBoxes = []
    for obstacle in obstacles:
        maxMin = findMaxMin(obstacle)
        maxMinBoxes.append(maxMin)
    return maxMinBoxes

def findMaxMin(obstacle):
    Xmin = float("inf")
    Ymin = float("inf")
    Xmax = -float("inf")
    Ymax = -float("inf")
    for corner in obstacle:
        if corner[0] < Xmin:
            Xmin = corner[0]
        if corner[0] > Xmax:
            Xmax = corner[0]
        if corner[1] < Ymin:
            Ymin = corner[1]
        if corner[1] > Ymax:
            Ymax = corner[1]
    return [Xmin, Xmax, Ymin, Ymax]


def checkIfHitBox(point, hitBoxes):
    posObstacles = []
    index = 0
    for box in hitBoxes:
        if box[0] <= point.x <= box[1] and box[2] <= point.y <= box[3]:
            posObstacles.append(index)
        index += 1
    return posObstacles


def isBlocked(node, obstacles, hitBoxes):
    posObsacles = checkIfHitBox(node, hitBoxes)
    if len(posObsacles) > 0:
        # There is a possibility that the point is within an obstacle
        for obstacleIndex in posObsacles:
            # Take point from outside of the box, now it is a line between the node and that point
            # x = minX -1.234, y = maxY, outside of the box (1.234 to minimize the risk of crossing a corner)
            outsideNode = [hitBoxes[obstacleIndex][0] - 1, hitBoxes[obstacleIndex][3]]
            obstacle = obstacles[obstacleIndex]
            if rayCastCheck(node, outsideNode, obstacle):
                return True
    return False


def rayCastCheck(node, outsideNode, obstacle):
    """"Obstacle walls 1-2, 2-3, 3-4, 4-1. Takes a point outside of the box
        Two lines intersect if (p1, q1, p2) and (p1, q1, q2) have different orientation AND
        (p2, q2, p1) and (p2, q2, q1) have different orientation. There is a special case not implemented yet.
        p1 = node to test, q1 = point outside box. p2 = corner in obstacle, q2 = connected corner
        Return True if node is inside of obstacle, otherwise return False
    """
    numIntersect = 0

    # Control so that it is only the corners that the ray potentially intersect that later gets checked
    posIntersectCorners = []
    for i in range(len(obstacle)):
        if isLinesCrossing(node.pos, outsideNode, obstacle[i], obstacle[(i+1)%len(obstacle)]):
            posIntersectCorners.append(i)
            numIntersect += 1


        #Old code
        #orientation1 = getOrientation(node.XY, outsideNode, obstacle[i])
        #orientation2 = getOrientation(node.XY, outsideNode, obstacle[(i+1) % len(obstacle)])
        #orientation3 = getOrientation(obstacle[i], obstacle[(i+1)%len(obstacle)], node.XY)
        #orientation4 = getOrientation(obstacle[i], obstacle[(i+1)%len(obstacle)], outsideNode)
        #if orientation1 != orientation2 and orientation3 != orientation4:
        #    posIntersectCorners.append(i)
        #    numIntersect += 1


    for i in posIntersectCorners:
        corner = obstacle[i]
        if isCornerOnLine(node.pos, outsideNode, corner):
            numIntersect -=1

    return numIntersect%2 != 0


def isLinesCrossing(node1, node2, node3, node4):
    """If the orientations is not the same, the lines are crossing"""
    orientation1 = getOrientation(node1, node2, node3)
    orientation2 = getOrientation(node1, node2, node4)
    orientation3 = getOrientation(node3, node4, node1)
    orientation4 = getOrientation(node3, node4, node2)
    return orientation1 != orientation2 and orientation3 != orientation4



def isCornerOnLine(point1, point2, point3):
    """Checks is point3 is on the line of point1-point2 using "cross product" """
    line1x = point3[0] - point1[0]
    line1y = point3[1] - point1[1]

    line2x = point2[0] - point1[0]
    line2y = point2[1] - point1[1]

    cross = line1x * line2y - line1y * line2x

    return cross == 0

def isPointOnSegment(point1, point2, point3):
    """Checks if point3 lies on the line point1-point2"""
    return min(point1[0], point2[0]) <= point3[0] <= max(point1[0], point2[0]) and \
           min(point1[1], point2[1]) <= point3[1] <= max(point1[1], point2[1])


def getOrientation(node1, node2, node3):
    """"Computes the orientation of the points. 0 = colinear, 1 = Clockwise, -1 = counterclockwise"""
    value = (node2[1] - node1[1]) * (node3[0] - node2[0]) - (node2[0] - node1[0]) * (node3[1] - node2[1])
    if value == 0:
        return 0
    return value / abs(value)

def isObstacleBetween(node1, node2, obstacles):
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            if isLinesCrossing(node1.pos, node2.pos, obstacle[i], obstacle[(i+1)%len(obstacle)]):
                return True

    return False





"""
    # Check for special cases is ray cast, not needed at the moment
    # node, outside, first corner is colinear and the first corner is on the line node-outside
    if orientation1 == 0 and isPointOnSegment(node.XY, outsideNode, obstacle[i]):
        numIntersect += 1
    # node, outside, second corner is colinear and the second corner is on the line node-outside
    if orientation2 == 0 and isPointOnSegment(node.XY, outsideNode, obstacle[(i + 1) % len(obstacle)]):
        numIntersect += 1
    # first corner, second corner, node is colinear and the node is on the line fist corner - second corner
    if orientation3 == 0 and isPointOnSegment(obstacle[i], obstacle[(i + 1) % len(obstacle)], node.XY):
        numIntersect += 1
    # first corner, second corner, outside is colinear and the outside is on the line first corner - second corner
    if orientation4 == 0 and isPointOnSegment(obstacle[i], obstacle[(i + 1) % len(obstacle)], outsideNode):
        numIntersect += 1
"""

