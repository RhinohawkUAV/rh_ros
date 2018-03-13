from geometry import lineSegment
from gui import Drawable
from gui import DrawableCircle

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
            lineSegment.drawLine(canvas, self.previousVertex._position, self._position, fill=lineColor, **kwargs)

    def draw(self, canvas, radius=DRAW_RADIUS, vertexColor="black", **kwargs):
        DrawableCircle(self._position[0], self._position[1], radius).draw(canvas, fill=vertexColor)
        canvas.create_text(self._position[0] + TEXT_OFFSET, self._position[1],
                           text="{:4.2f}".format(self._timeCost),
                           fill=vertexColor)
