import gui.draw
from gui import Drawable

DRAW_RADIUS = 0.5
TEXT_OFFSET = 3.0


class Vertex(Drawable):
    def __init__(self, position, time, velocity, previousVertex=None):
        # The location of the vertex
        self.position = position

        # The time from start through this vertex
        self.time = time

        # The incoming velocity vector when arriving at this point
        self.velocity = velocity

        # The previous vertex which is part of the shortest path from start through this vertex
        self.previousVertex = previousVertex

    def drawPath(self, canvas, **kwargs):
        if not self.previousVertex is None:
            gui.draw.drawLine(canvas, self.previousVertex.position, self.position, **kwargs)
            self.previousVertex.drawPath(canvas, **kwargs)

    def draw(self, canvas, **kwargs):
        gui.draw.drawPoint(canvas, self.position, **kwargs)
        gui.draw.drawText(canvas, (self.position[0] + TEXT_OFFSET, self.position[1]),
                          text="{:4.2f}".format(self.time), **kwargs)

    def drawEdge(self, canvas, **kwargs):
        if not self.previousVertex is None:
            gui.draw.drawLine(canvas, self.previousVertex.position, self.position, **kwargs)

    def __str__(self):
        return "(" + str(self.position[0]) + "," + str(self.position[1]) + ") with cost: " + str(self.time)
