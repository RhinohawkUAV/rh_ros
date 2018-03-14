import copy
import math
import time
from Tkinter import Canvas

import numpy as np

import gui
from gui import Drawable
from findPathStatic.staticSearchGraph import StaticSearchGraph


class StaticPathFinder(Drawable):
    """
    Holds entire geometric state of the system, including, no-fly zones, their velocities, etc.
    Path finding related queries can be be performed on this object.
    """

    def __init__(self, startPoint, goalPoint, obstacleCourse, drawListener=None):

        # Start position to search from
        self._startPoint = startPoint

        # End/goal position to find
        self._goalPoint = goalPoint

        self._goalArray = np.array(goalPoint, np.double)

        self._speed = 1.0

        # The obstacle course to traverse
        self._obstacleCourse = obstacleCourse

        # A staticGraph object for perform the staticGraph algorithm
        self._graph = StaticSearchGraph(self._startPoint)

        # Current vertex in the search algorithm
        self._currentVertex = None

        # Visible _points from current position being examined
        self._visiblePoints = []

        # This object is signalled, with a copy of findPathStatic, whenever it is time to draw
        self.drawListener = drawListener

    def findPath(self):
        """
        Does the pathfinding algorithm
        :return:
        """

        visibleCalcTime = 0
        totalCalcTime = 0

        self._currentVertex = self._graph.getNextVertex()
        self._currentPoint = np.array(self._currentVertex.data, np.double)
        while not self._currentVertex is None and not self._currentVertex.data == self._goalPoint:
            totalCalcTime -= time.time()

            visibleCalcTime -= time.time()
            paths = self._obstacleCourse.findStraightPathsToVertices(self._currentPoint, self._speed)
            self._visiblePoints = []

            for path in paths:
                self._visiblePoints.append(tuple(path.destination))

            if not self._obstacleCourse.doesLineIntersect(self._currentPoint, self._goalArray, self._speed):
                self._visiblePoints.append(self._goalPoint)
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
            self._currentPoint = np.array(self._currentVertex.data, np.double)
            totalCalcTime += time.time()

        # Visible _points will be drawn wrong if not recomputed AND they are irrelevant (we are at the goal)
        self._visiblePoints = []
        self._graph.setEmphasizedPathEnd(self._goalPoint)
        self._signalDraw()

        # Note: 99.9% time is currently taken up in computing visibility.  This can be vastly improved if necessary.
        print "Total Calculation Time: " + str(totalCalcTime)
        print "Total Time Calculating Visibility: " + str(visibleCalcTime)
        print "Visibility/Total: " + str(visibleCalcTime / totalCalcTime)

    def _signalDraw(self):
        if not self.drawListener is None:
            drawCopy = self._createDrawCopy()
            self.drawListener.onDraw(drawCopy)
            # Throttle drawing rate.  Haven't researched how to set a dirty bit on the canvas and force a redraw.
            # Currently just submit an endless queue of drawings, which can pile up.
            time.sleep(0.1)

    def _createDrawCopy(self):
        """
        Creates a copy of the findPathStatic object for drawing.
        Its not possible to deepcopy tkinter objects referenced under the self.drawListener field.
        This creates a copy, for drawing, which does not include that field.
        """
        drawListener = self.drawListener
        self.drawListener = None
        copyObject = copy.deepcopy(self)
        self.drawListener = drawListener
        return copyObject

    def draw(self, canvas, time=0.0, **kwargs):
        # type: (Canvas)->None
        """Must be called from GUI thread"""

        self._obstacleCourse.draw(canvas, time=time, drawVectors=False, **kwargs)

        for visiblePoint in self._visiblePoints:
            gui.draw.drawLine(canvas, self._currentVertex.data, visiblePoint, color="blue")

        gui.draw.drawPoint(canvas, self._currentVertex.data, color="green")
        gui.draw.drawPoint(canvas, self._startPoint, color="green")
        gui.draw.drawPoint(canvas, self._goalPoint, color="green")

        self._graph.draw(canvas, **kwargs)
