import vertex
from render.Drawable import Drawable
from render.drawables import DrawableLine
from vertex import Vertex


class SearchGraph(Drawable):
    """
    A graph used to find the shortest path.  Each vertex holds some piece of metadata and can be looked up using the information.
    """

    def __init__(self, startData):
        self._unvisitedVertices = {}
        self._visitedVertices = {}
        self._emphasizedPathEnd = None
        self._addUnvisited(startData, 0)

    def getUnvisitedVertex(self, data):
        return self._unvisitedVertices[data]

    def getVisitedVertex(self, data):
        return self._visitedVertices[data]

    def beenVisited(self, data):
        return self._visitedVertices.has_key(data)

    def updateCost(self, sourceVertex, data, cost):
        """
        A route has been found from sourceVertex, to the vertex identified by data (if it exists), with the given cost.

        If no vertex corresponds to data, then we just discovered one.  Create and add it.

        If a vertex already exists at data, then update its cost, if appropriate.

        :param data:
        :param sourceVertex:
        :param cost:
        :return:
        """
        totalCost = cost + sourceVertex.totalCost
        if self._unvisitedVertices.has_key(data):
            existingVertex = self.getUnvisitedVertex(data)

            if existingVertex.totalCost > totalCost:
                existingVertex.totalCost = totalCost
                existingVertex.previous = sourceVertex

        else:
            self._addUnvisited(data, totalCost, sourceVertex)

    def _addUnvisited(self, data, totalCost, previousVertex=None):
        """
        Creates a new vertex with the given metadata, totalCost, and possible previousVertex.
        :param data:
        :param totalCost:
        :param previousVertex:
        :return:
        """
        self._unvisitedVertices[data] = Vertex(data, totalCost, previousVertex)

    def getNextVertex(self):
        if len(self._unvisitedVertices.values()) == 0:
            return None

        minVertex = vertex.minCostVertex(self._unvisitedVertices.values())
        self._visitedVertices[minVertex.data] = minVertex
        del self._unvisitedVertices[minVertex.data]
        return minVertex

    def traverseLeastPath(self, end, func):
        vertex = end
        while not vertex is None:
            func(vertex)
            vertex = vertex.previous

    def setEmphasizedPathEnd(self, endPoint):
        """Sets the end point of the shortest path to emphasize"""
        self._emphasizedPathEnd = self.getVisitedVertex(endPoint)

    def drawLeastPath(self, end, canvas):
        def drawLine(vertex):
            if not vertex.previous is None:
                line = DrawableLine(vertex.data[0], vertex.data[1],
                                    vertex.previous.data[0], vertex.previous.data[1],
                                    width=4,
                                    fill="orange")
                line.draw(canvas)

        self.traverseLeastPath(end, drawLine)

    def draw(self, canvas):
        for vertex in self._unvisitedVertices.values():
            vertex.drawAsVisited = False
            vertex.draw(canvas)
        for vertex in self._visitedVertices.values():
            vertex.drawAsVisited = True
            vertex.draw(canvas)

        self.drawLeastPath(self._emphasizedPathEnd, canvas)


class Edge:
    def __init__(self, p1, p2, distance):
        self.p1 = p1
        self.p2 = p2
        self.distance = distance
