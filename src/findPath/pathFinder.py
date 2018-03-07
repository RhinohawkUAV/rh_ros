import copy
import math
import time
from Tkinter import Canvas

from shapely.geometry import LineString

from graph.graph import SearchGraph
from gui import Drawable
from gui import DrawableLine, DrawableCircle


class PathFinder(Drawable):
    """
    Holds entire geometric state of the system, including, no-fly zones, their velocities, etc.
    Path finding related queries can be be performed on this object.
    """

    def __init__(self, startPoint, goalPoint, noFlyZones, drawListener=None):

        # Start position to search from
        self._startPoint = startPoint

        # End/goal position to find
        self._goalPoint = goalPoint

        # The NFZs to avoid
        self._noFlyZones = noFlyZones

        # A graph object for perform the graph algorithm
        self._graph = SearchGraph(self._startPoint)

        # Current vertex in the search algorithm
        self._currentVertex = None

        # Visible points from current position being examined
        self._visiblePoints = []

        # This object is signalled, with a copy of findPath, whenever it is time to draw
        self.drawListener = drawListener

    def findPath(self):
        """
        Does the pathfinding algorithm
        :return:
        """

        visibleCalcTime = 0
        totalCalcTime = 0

        self._currentVertex = self._graph.getNextVertex()
        while not self._currentVertex is None and not self._currentVertex.data is self._goalPoint:
            totalCalcTime -= time.time()

            visibleCalcTime -= time.time()
            self._visiblePoints = self._findVisibleVertices(self._currentVertex.data)
            visibleCalcTime += time.time()

            for visiblePoint in self._visiblePoints:
                if not self._graph.beenVisited(visiblePoint):
                    x = self._currentVertex.data[0] - visiblePoint[0]
                    y = self._currentVertex.data[1] - visiblePoint[1]
                    distance = math.sqrt(x * x + y * y)

                    self._graph.updateCost(self._currentVertex, visiblePoint, distance)

            self._graph.setEmphasizedPathEnd(self._currentVertex.data)
            totalCalcTime += time.time()

            self._signalDraw()

            totalCalcTime -= time.time()
            self._currentVertex = self._graph.getNextVertex()
            totalCalcTime += time.time()

        # Visible points will be drawn wrong if not recomputed AND they are irrelevant (we are at the goal)
        self._visiblePoints = []
        self._graph.setEmphasizedPathEnd(self._goalPoint)
        self._signalDraw()

        # Note: 99.9% time is currently taken up in computing visibility.  This can be vastly improved if necessary.
        print "Total Calculation Time: " + str(totalCalcTime)
        print "Total Time Calculating Visibility: " + str(visibleCalcTime)
        print "Visibility/Total: " + str(visibleCalcTime / totalCalcTime)

    def _isPointVisible(self, eye, point):
        """
        Is the given point visible from the given eye?
        In other words is there a straight path from eye to point which does not intersect any no-fly-zones?
        """
        line = LineString([eye, point])
        for noFlyZone in self._noFlyZones:
            if noFlyZone.blocksLineOfSight(line):
                return False
        return True

    def _findVisibleVertices(self, eye):
        """
        Look through all NFZ vertices and the goal and determine which are visible from eye.
        """
        visibleVertices = []
        for noFlyZone in self._noFlyZones:
            for polygonVertex in noFlyZone.points:
                if self._isPointVisible(eye, polygonVertex):
                    visibleVertices.append(polygonVertex)
        if self._isPointVisible(eye, self._goalPoint):
            visibleVertices.append(self._goalPoint)

        return visibleVertices

    def _signalDraw(self):
        if not self.drawListener is None:
            drawCopy = self._createDrawCopy()
            self.drawListener.onDraw(drawCopy)
            # Throttle drawing rate.  Haven't researched how to set a dirty bit on the canvas and force a redraw.
            # Currently just submit an endless queue of drawings, which can pile up.
            time.sleep(0.1)

    def _createDrawCopy(self):
        """
        Creates a copy of the findPath object for drawing.
        Its not possible to deepcopy tkinter objects referenced under the self.drawListener field.
        This creates a copy, for drawing, which does not include that field.
        """
        drawListener = self.drawListener
        self.drawListener = None
        copyObject = copy.deepcopy(self)
        self.drawListener = drawListener
        return copyObject

    def draw(self, canvas, **kwargs):
        # type: (Canvas)->None
        """Must be called from GUI thread"""

        for noFlyZone in self._noFlyZones:
            noFlyZone.draw(canvas, **kwargs)

        for visiblePoint in self._visiblePoints:
            DrawableLine(self._currentVertex.data[0], self._currentVertex.data[1], visiblePoint[0],
                         visiblePoint[1]).draw(canvas, fill="blue")

        DrawableCircle(self._currentVertex.data[0], self._currentVertex.data[1], 1.0).draw(canvas, fill="green")
        DrawableCircle(self._startPoint[0], self._startPoint[1], 1.0).draw(canvas, fill="green")
        DrawableCircle(self._goalPoint[0], self._goalPoint[1], 1.0).draw(canvas, fill="green")

        self._graph.draw(canvas, **kwargs)
