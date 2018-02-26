from render.Drawable import Drawable


class VertexBag(Drawable):
    def __init__(self):
        self.vertices = {}
        self.visited = {}
        self.edges = {}

    def putVertex(self, vertex):
        self.vertices[vertex.position] = vertex

    def getVertex(self, position):
        return self.vertices[position]

    def updateVertex(self, position, shortestDistance):

        if self.vertices.has_key(position):
            existingVertex = self.getVertex(position)
            if existingVertex.shortestDistance > shortestDistance:
                existingVertex.shortestDistance = shortestDistance
        else:
            existingVertex = Vertex(position, shortestDistance)
            self.putVertex(existingVertex)

    def beenVisited(self, position):
        return self.visited.has_key(position)

    def hasUnvisted(self):
        return len(self.vertices) > 0

    def getLowestCostUnvisted(self):
        minVertex = self.vertices.values()[0]
        i = 1

        vertexList = self.vertices.values()
        while i < len(vertexList):
            if vertexList[i].shortestDistance < minVertex.shortestDistance:
                minVertex = vertexList[i]
            i += 1

        self.visited[minVertex.position] = minVertex
        del self.vertices[minVertex.position]
        return minVertex

    def putEdge(self, p1, p2, distance):
        self.edges[(p1, p2)] = Edge(p1, p2, distance)

    def draw(self, canvas):
        for vertex in self.vertices.values():
            canvas.create_text(vertex.position[0], vertex.position[1], text='{:5.2f}'.format(vertex.shortestDistance),
                               fill="black")
        for vertex in self.visited.values():
            canvas.create_text(vertex.position[0], vertex.position[1], text='{:5.2f}'.format(vertex.shortestDistance),
                               fill="blue")

        for edge in self.edges.values():
            midPoint = ((edge.p1[0] + edge.p2[0]) / 2, (edge.p1[1] + edge.p2[1]) / 2)

            canvas.create_text(midPoint[0], midPoint[1], text='{:5.2f}'.format(edge.distance), fill="black")
            canvas.create_line(edge.p1[0], edge.p1[1], edge.p2[0], edge.p2[1], fill="green")


class Vertex:
    def __init__(self, position, shortestDistance):
        self.position = position
        self.shortestDistance = shortestDistance


class Edge:
    def __init__(self, p1, p2, distance):
        self.p1 = p1
        self.p2 = p2
        self.distance = distance
