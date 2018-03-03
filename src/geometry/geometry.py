import copy
import math
import time
from Tkinter import Canvas

from shapely.geometry import LineString
from typing import List

from graph.graph import VertexBag, Vertex
from noFlyZone import NoFlyZone
from render.drawGroup import DrawGroup
from render.drawables import DrawableLine, DrawableCircle


class Geometry(DrawGroup):
    """
    Holds entire geometric state of the system, including, no-fly zones, their velocities, etc.
    Path finding related queries can be be performed on this object.
    """

    def __init__(self, noFlyZones):
        # type: (List[NoFlyZone]) -> None

        DrawGroup.__init__(self)
        self.noFlyZones = noFlyZones

    def isPositionVisible(self, eye, position):
        """Is the given position visible from eye?"""
        line = LineString([eye, position])
        for noFlyZone in self.noFlyZones:
            if noFlyZone.blocksLineOfSight(line):
                return False
        return True

    def findVisibleVertices(self, eye, end):
        """Find all no fly zon vertices + end, which are visible from eye"""
        visibleVertices = []
        for noFlyZone in self.noFlyZones:
            for polygonVertex in noFlyZone.vertices:
                if self.isPositionVisible(eye, polygonVertex):
                    visibleVertices.append(polygonVertex)
        if self.isPositionVisible(eye, end):
            visibleVertices.append(end)

        return visibleVertices

    def findPath(self, startPoint, endPoint, renderer):
        self.graph = VertexBag()
        self.graph.putVertex(Vertex(startPoint, 0))

        while self.graph.hasUnvisted():
            vertex = self.graph.getNextVertex()

            if vertex.position is endPoint:
                self.graph.setShortestPathStart(endPoint)
                break

            visiblePoints = self.findVisibleVertices(vertex.position, endPoint)

            for visiblePoint in visiblePoints:
                if not self.graph.beenVisited(visiblePoint):
                    x = vertex.position[0] - visiblePoint[0]
                    y = vertex.position[1] - visiblePoint[1]
                    distance = math.sqrt(x * x + y * y)

                    self.graph.updateVertex(visiblePoint, vertex, distance)

            # Drawing
            for visiblePoint in visiblePoints:
                drawLine = DrawableLine(vertex.position[0], vertex.position[1], visiblePoint[0], visiblePoint[1],
                                        fill="blue")
                self.addDrawable(drawLine)

            self.graph.setShortestPathStart(vertex.position)
            self.addDrawable(DrawableCircle(vertex.position[0], vertex.position[1], 1.0, fill="green"))
            self.addDrawable(DrawableCircle(endPoint[0], endPoint[1], 1.0, fill="green"))
            renderCopy = copy.deepcopy(self)
            renderer.render(renderCopy)
            self.clearDrawables()
            time.sleep(0.1)

        self.clearDrawables()
        self.addDrawable(DrawableCircle(startPoint[0], startPoint[1], 1.0, fill="green"))
        self.addDrawable(DrawableCircle(endPoint[0], endPoint[1], 1.0, fill="green"))
        renderCopy = copy.deepcopy(self)
        renderer.render(renderCopy)

    def draw(self, canvas):
        # type: (Canvas)->None
        """Must be called from GUI thread"""

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas)

        DrawGroup.draw(self, canvas)
        self.graph.draw(canvas)
