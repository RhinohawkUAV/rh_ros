from render.Drawable import Drawable
from render.drawables import DrawableLine, DrawableCircle


class VertexBag(Drawable):
    def __init__(self):
        self.unvisitedVertices = {}
        self.visitedVertices = {}
        self.edges = {}
        self.shortestPathStart = None

    def putVertex(self, vertex):
        self.unvisitedVertices[vertex.position] = vertex

    def getVertex(self, position):
        return self.unvisitedVertices[position]

    def updateVertex(self, position, sourceVertex, distance):

        totalDistance = distance + sourceVertex.shortestDistance
        if self.unvisitedVertices.has_key(position):
            existingVertex = self.getVertex(position)

            if existingVertex.shortestDistance > totalDistance:
                existingVertex.shortestDistance = totalDistance
                existingVertex.previousShortest = sourceVertex

        else:
            existingVertex = Vertex(position, totalDistance)
            existingVertex.previousShortest = sourceVertex
            self.putVertex(existingVertex)

    def beenVisited(self, position):
        return self.visitedVertices.has_key(position)

    def hasUnvisted(self):
        return len(self.unvisitedVertices) > 0

    def minLowestCostVertex(self, vertices):
        minVertex = vertices[0]
        i = 1

        while i < len(vertices):
            if vertices[i].shortestDistance < minVertex.shortestDistance:
                minVertex = vertices[i]
            i += 1

        return minVertex

    def getNextVertex(self):
        minVertex = self.minLowestCostVertex(self.unvisitedVertices.values())
        self.visitedVertices[minVertex.position] = minVertex
        del self.unvisitedVertices[minVertex.position]
        return minVertex

    def putEdge(self, p1, p2, distance):
        self.edges[(p1, p2)] = Edge(p1, p2, distance)

    def traverseLeastPath(self, end, func):
        vertex = end
        while not vertex is None:
            func(vertex)
            vertex = vertex.previousShortest

    def drawLeastPath(self, end, canvas):
        def drawLine(vertex):
            if not vertex.previousShortest is None:
                line = DrawableLine(vertex.position[0], vertex.position[1],
                                    vertex.previousShortest.position[0], vertex.previousShortest.position[1],
                                    width=4,
                                    fill="orange")
                line.draw(canvas)

        self.traverseLeastPath(end, drawLine)

    def draw(self, canvas):
        for vertex in self.unvisitedVertices.values():
            point = DrawableCircle(vertex.position[0], vertex.position[1], 0.5, fill="purple")
            point.draw(canvas)
            canvas.create_text(vertex.position[0] + 3, vertex.position[1],
                               text="{:4.2f}".format(vertex.shortestDistance),
                               fill="black")
        for vertex in self.visitedVertices.values():
            if not vertex.previousShortest is None:
                line = DrawableLine(vertex.position[0], vertex.position[1],
                                    vertex.previousShortest.position[0], vertex.previousShortest.position[1],
                                    width=2,
                                    fill="green")
                line.draw(canvas)

        self.drawLeastPath(self.shortestPathStart, canvas)

    def setShortestPathStart(self, endPoint):
        self.shortestPathStart = self.visitedVertices[endPoint]


class Vertex:
    def __init__(self, position, shortestDistance):
        # 2D position of this vertex in
        self.position = position

        # The shortest path to this vertex from the start vertex
        self.shortestDistance = shortestDistance

        # The previous vertex which is part of the shortest path from start to this vertex
        self.previousShortest = None


class Edge:
    def __init__(self, p1, p2, distance):
        self.p1 = p1
        self.p2 = p2
        self.distance = distance
