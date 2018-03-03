import copy
import math
import time
from Tkinter import Canvas

from shapely.geometry import LineString

from graph.graph import VertexBag, Vertex
from render.Drawable import Drawable
from render.drawables import DrawableLine, DrawableCircle


class PathFinderVisualizer():
    """
    Visualizes a Geometry object during the path finding process.  Installs itself as a listener and gets a drawable
    copy of the geometry object on a regular basis.
    """

    def __init__(self, geometry, renderer):
        self.geometry = geometry
        self.geometry.drawListener = self
        self.renderer = renderer

    def drawGeometry(self, geometry):
        # Drawing
        if not self.renderer is None:
            renderCopy = copy.deepcopy(geometry)
            self.renderer.render(renderCopy)


class PathFinder(Drawable):
    """
    Holds entire geometric state of the system, including, no-fly zones, their velocities, etc.
    Path finding related queries can be be performed on this object.
    """

    def __init__(self, startPoint, goalPoint, noFlyZones, drawListener=None):

        # Start position to search from
        self.startPoint = startPoint

        # End/goal position to find
        self.goalPoint = goalPoint

        # The NFZs to avoid
        self.noFlyZones = noFlyZones

        # This object is signalled, with a copy of geometry, whenever it is time to draw
        self.drawListener = drawListener

        # Visible points from current position being examined
        self.visiblePoints = []

        # Current vertex in the search algorithm
        self.currentVertex = None

    def isPointVisible(self, eye, point):
        """
        Is the given point visible from the given eye?
        In other words is there a straight path from eye to point which does not intersect any no-fly-zones?
        """
        line = LineString([eye, point])
        for noFlyZone in self.noFlyZones:
            if noFlyZone.blocksLineOfSight(line):
                return False
        return True

    def findVisibleVertices(self, eye):
        """
        Look through all NFZ vertices and the goal and determine which are visible from eye.
        """
        visibleVertices = []
        for noFlyZone in self.noFlyZones:
            for polygonVertex in noFlyZone.vertices:
                if self.isPointVisible(eye, polygonVertex):
                    visibleVertices.append(polygonVertex)
        if self.isPointVisible(eye, self.goalPoint):
            visibleVertices.append(self.goalPoint)

        return visibleVertices

    def findPath(self, renderer):
        self.graph = VertexBag()
        self.graph.putVertex(Vertex(self.startPoint, 0))

        visibleCalcTime = 0
        totalCalcTime = 0
        while self.graph.hasUnvisted():
            totalCalcTime -= time.time()
            self.currentVertex = self.graph.getNextVertex()

            if self.currentVertex.position is self.goalPoint:
                self.graph.setShortestPathStart(self.goalPoint)
                totalCalcTime += time.time()
                break
            else:
                self.graph.setShortestPathStart(self.currentVertex.position)

            visibleCalcTime -= time.time()
            self.visiblePoints = self.findVisibleVertices(self.currentVertex.position)
            visibleCalcTime += time.time()

            for visiblePoint in self.visiblePoints:
                if not self.graph.beenVisited(visiblePoint):
                    x = self.currentVertex.position[0] - visiblePoint[0]
                    y = self.currentVertex.position[1] - visiblePoint[1]
                    distance = math.sqrt(x * x + y * y)

                    self.graph.updateVertex(visiblePoint, self.currentVertex, distance)

            totalCalcTime += time.time()

            if not self.drawListener is None:
                drawCopy = self.createDrawCopy()
                self.drawListener.drawGeometry(drawCopy)
                #Throttle drawing rate.  Haven't researched how to set a dirty bit on the canvas and force a redraw.
                # Currently just submit an endless queue of drawings, which can pile up.
                time.sleep(0.1)

        if not self.drawListener is None:
            drawCopy = self.createDrawCopy()
            self.drawListener.drawGeometry(drawCopy)

        print "Total Calculation Time: " + str(totalCalcTime)
        print "Total Time Calculating Visibility: " + str(visibleCalcTime)
        print "Visibility/Total: " + str(visibleCalcTime / totalCalcTime)

    def draw(self, canvas):
        # type: (Canvas)->None
        """Must be called from GUI thread"""

        for noFlyZone in self.noFlyZones:
            noFlyZone.draw(canvas)

        for visiblePoint in self.visiblePoints:
            drawLine = DrawableLine(self.currentVertex.position[0], self.currentVertex.position[1], visiblePoint[0],
                                    visiblePoint[1],
                                    fill="blue")
            drawLine.draw(canvas)

        DrawableCircle(self.currentVertex.position[0], self.currentVertex.position[1], 1.0, fill="green").draw(canvas)
        DrawableCircle(self.startPoint[0], self.startPoint[1], 1.0, fill="green").draw(canvas)
        DrawableCircle(self.goalPoint[0], self.goalPoint[1], 1.0, fill="green").draw(canvas)

        self.graph.draw(canvas)

    def createDrawCopy(self):
        """
        Creates a copy of the geometry object for drawing.
        Its not possible to deepcopy tkinter objects referenced under the self.drawListener field.
        This creates a copy, for drawing, which does not include that field.
        """
        drawListener = self.drawListener
        self.drawListener = None
        copyObject = copy.deepcopy(self)
        self.drawListener = drawListener
        return copyObject
