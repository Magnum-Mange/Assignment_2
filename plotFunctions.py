import matplotlib.pyplot as plt
import queue

def plotCorners(obstacle, color):
    for cor in range(len(obstacle)):
        c1 = obstacle[cor]
        c2 = obstacle[(cor + 1) % len(obstacle)]
        plt.plot([c1[0], c2[0]], [c1[1], c2[1]], c = color)


def plotMap(polygon):
    plotCorners(polygon, "k")
    plt.axis("equal")


def plotTree(startNode):
     q = queue.Queue()
     q.put(startNode)
     while not q.empty():
         currentNode = q.get()
         for child in currentNode.children:
             q.put(child)
             plt.plot([currentNode.x, child.x], [currentNode.y, child.y], c = "b")

def plotPath(endNode):
    node = endNode
    while not node.parent == None:
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], c = "g")
        node = node.parent


