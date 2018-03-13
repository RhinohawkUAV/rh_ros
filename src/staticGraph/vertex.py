from gui import Drawable
from gui import DrawableLine, DrawableCircle


def minCostVertex(vertices):
    minVertex = vertices[0]
    i = 1

    while i < len(vertices):
        if vertices[i].totalCost < minVertex.totalCost:
            minVertex = vertices[i]
        i += 1

    return minVertex


class Vertex(Drawable):
    def __init__(self, data, totalCost, previous=None):
        # The metadata associated with this vertex
        self.data = data

        # The lowest total cost from start to this vertex, found so far
        self.totalCost = totalCost

        # The previous vertex which is part of the shortest path from start to this vertex
        self.previous = previous

        # Tracked for drawing purposes only (not required for algorithm)
        self.drawAsVisited = False

    def draw(self, canvas, **kwargs):
        if self.drawAsVisited:
            if not self.previous is None:
                line = DrawableLine(self.data[0], self.data[1],
                                    self.previous.data[0], self.previous.data[1])
                line.draw(canvas, width=2, fill="green")
        else:
            DrawableCircle(self.data[0], self.data[1], 0.5).draw(canvas, fill="purple")
            canvas.create_text(self.data[0] + 3, self.data[1],
                               text="{:4.2f}".format(self.totalCost),
                               fill="black")
