import math

import constants
from engine.geometry import calcs
from engine.vertex.uniqueVertexQueue.uniqueSpaceTree import UniqueTree
from engine.vertex.vertexPriorityQueue import VertexPriorityQueue, \
    QueueEmptyException
import numpy as np
from utils.minheap import MinHeap


# TODO: Remove high number of 0 uniqueness points
# TODO: Is diagonal length useful?
class UniqueVertexQueue(VertexPriorityQueue):
    """
    This VertexPriorityQueue uses several criteria to determine priority:

    1. Estimated time - The smaller a vertex's estimatedTimeThroughVertex field is, the higher its priority.
    2. Uniqueness - The more unique a vertex is the higher its priority.

    Uniqueness depends on position and velocity of a vertex in comparison to other vertices and is given a score [0,1]:
    1 - this is the only vertex
    0 - this exactly matches an existing vertex


    We define priority (lower is better) as:
    _diagnonalTime = sqrt(width*width + height*height) / maximumSpeed
    priority = estimatedTime - uniqueness * _diagnonalTime

    The goal is to scale uniqueness so that it is on the same playing field as estimated time.

    Uniqueness is judged at the time the vertex is discovered.  This achieves our goal as the first vertex in an
    area/configuration will have a high priority, but later ones will have lesser priority.

    #TODO: Future tuning considerations:
    1. Should uniqueness of velocity be given the same weight as position?
    2. Should uniqueness be given more or less weight compared to estimated time?
    """

    def __init__(self, x, y, width, height, maximumSpeed, numWaypoints, coincidentDistance=constants.UNIQUE_TREE_COINCIDENT_DISTANCE):
        self._heap = MinHeap()
        self._uniqueTrees = []
        for i in range(numWaypoints):
            self._uniqueTrees.append(UniqueTree(
                        minPosition=(x, y, -maximumSpeed, -maximumSpeed),
                        dims=(width, height, 2.0 * maximumSpeed, 2.0 * maximumSpeed),
                        coincidentDistance=coincidentDistance))
#        self._diagonal = calcs.length(np.array((width, height), np.double))

    def push(self, vertex):
        vPos = vertex.getPosition()
        vVel = vertex.getVelocity()
        treeIndex = vertex.getWaypoint().getIndex()
        uniqueness = self._uniqueTrees[treeIndex].insert(position=np.array([vPos[0], vPos[1], vVel[0], vVel[1]]))

#         length = 1.0 + calcs.length(vertex.getWaypoint()._position - vPos) / self._diagonal
#         distBias = math.pow(length, 0.25)
        
        if uniqueness == 0.0:
            return
        priority = vertex.getTimeThroughPriority() / uniqueness
        
        self._heap.push(priority, vertex)

    def pop(self):
        if self._heap.isEmpty():
            raise QueueEmptyException
        return self._heap.pop()

    def isEmpty(self):
        return self._heap.isEmpty()

    def __iter__(self):
        """Should allow iteration over vertices"""
        return self._heap.__iter__()
