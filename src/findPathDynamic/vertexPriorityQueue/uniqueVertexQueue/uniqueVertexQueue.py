import numpy as np

from findPathDynamic.vertexPriorityQueue.vertexPriorityQueue import VertexPriorityQueue
from uniqueVertexTree import UniqueTree
from utils.minheap import MinHeap


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

    def __init__(self, x, y, width, height, maximumSpeed):
        self._heap = MinHeap()
        self._uniqueTree = UniqueTree(x, y, width, height, maximumSpeed)
        self._diagnonalTime = np.math.sqrt(width * width + height * height) / maximumSpeed

    def push(self, vertex):
        uniqueness = self._uniqueTree.insert(vertex)
        priority = vertex.estimatedTimeThroughVertex - uniqueness * self._diagnonalTime
        self._heap.push(priority, vertex)

    def pop(self):
        if self._heap.isEmpty():
            return None
        return self._heap.pop()

    def isEmpty(self):
        return self._heap.isEmpty()

    def __iter__(self):
        """Should allow iteration over vertices"""
        return self._heap.__iter__()
