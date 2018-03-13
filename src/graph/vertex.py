import gui.draw
from gui import Drawable

DRAW_RADIUS = 0.5
TEXT_OFFSET = 3.0


class Vertex(Drawable):
    def __init__(self, timeCost, position, velocity, previousVertex=None):
        # The lowest total timeCost from start to this vertex, found so far.  Time=Cost
        self._timeCost = timeCost

        # The location of the vertex
        self._position = position

        # The incoming velocity vector when arriving at this point
        self._velocity = velocity

        # The previous vertex which is part of the shortest path from start through this vertex
        self.previousVertex = previousVertex

    def drawPath(self, canvas, lineColor="black", **kwargs):
        self.draw(canvas, **kwargs)
        if not self.previousVertex is None:
            self.previousVertex.draw(self, canvas, lineColor=lineColor, **kwargs)
            gui.draw.drawLine(canvas, self.previousVertex._position, self._position, color=lineColor, **kwargs)

    def draw(self, canvas, **kwargs):
        gui.draw.drawPoint(canvas, self._position, **kwargs)
        gui.draw.drawText(canvas, (self._position[0] + TEXT_OFFSET, self._position[1]),
                          text="{:4.2f}".format(self._timeCost), **kwargs)
