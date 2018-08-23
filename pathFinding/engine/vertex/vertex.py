from gui import Drawable
import gui.draw

DRAW_RADIUS = 0.5
TEXT_OFFSET = 3.0


class Vertex(Drawable):

    def __init__(self, position, startSpeed, unitVelocity, timeToVertex, estimatedTimeThroughVertex, previousVertex=None,
                 pathSegment=None):
        # The location of the vertex
        self.position = position

        # The incoming velocity vector when arriving at this point
        self.speed = startSpeed
        self.unitVelocity = unitVelocity

        # The timeToVertex from start to this vertex
        self.timeToVertex = timeToVertex

        # An estimated of the timeToVertex required to traverse, the best possible path from start, through this vertex, to goal
        self.estimatedTimeThroughVertex = estimatedTimeThroughVertex
        if pathSegment is None:
            self.nextLegalRotDirection = 0
        else:
            # The direction which an object was previously skirted to get here
            self.nextLegalRotDirection = pathSegment.nextLegalRotDirection

        # The previous vertex which is part of the shortest path from start through this vertex
        self.previousVertex = previousVertex

        # A path segment from the previous vertex to this vertex.  Stored for rendering/debug.
        self.pathSegment = pathSegment

        # Potentially holds information for debugging
        self.debug = None

    def drawPath(self, canvas, **kwargs):
        if self.previousVertex is not None:
            self.pathSegment.draw(canvas, **kwargs)
            self.previousVertex.drawPath(canvas, **kwargs)

    def draw(self, canvas, **kwargs):
        gui.draw.drawPoint(canvas, self.position, **kwargs)
        gui.draw.drawText(canvas, (self.position[0] + TEXT_OFFSET, self.position[1]),
                          text="{:4.2f}".format(self.timeToVertex), **kwargs)

    def drawEdge(self, canvas, **kwargs):
        if not self.previousVertex is None:
            gui.draw.drawLine(canvas, self.previousVertex.position, self.position, **kwargs)

    def __str__(self):
        return "(" + str(self.position[0]) + "," + str(self.position[1]) + ") with cost: " + str(self.timeToVertex)
