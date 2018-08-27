from gui import Drawable
import gui.draw
import numpy as np

DRAW_RADIUS = 0.5
TEXT_OFFSET = 3.0
_zero = np.array((0, 0), np.double)


class BaseVertex(Drawable):

    def __init__(self, waypoint):
        self._waypoint = waypoint

    def getWaypoint(self):
        return self._waypoint

    def getPosition(self):
        pass

    def getVelocity(self):
        pass

    def getTimeTo(self):
        pass
    
    def getTimeThroughAdmissible(self):
        pass

    def getTimeThroughPriority(self):
        pass

    def getPreviousVertex(self):
        pass

    def pathSegmentsToWaypoint(self, obstacleCourse):
        pass

    def skirtingPathSegments(self, obstacleCourse):
        pass

    def drawPath(self, canvas, **kwargs):
        pass

    def draw(self, canvas, **kwargs):
        pass

    
class OriginVertex(BaseVertex):

    def __init__(self, waypoint, position, speed, direction):
        BaseVertex.__init__(self, waypoint)
        self._position = position
        self._direction = direction
        self._speed = speed

    def getPosition(self):
        return self._position

    def getVelocity(self):
        return self._direction * self._speed

    def getTimeTo(self):
        return 0.0
    
    def getTimeThroughAdmissible(self):
        # Irrelevant value as this vertex will never have any competition
        return 0.0

    def getTimeThroughPriority(self):
        # Irrelevant value as this vertex will never have any competition
        return 0.0
    
    def getPreviousVertex(self):
        return None

    def pathSegmentsToWaypoint(self, obstacleCourse): 
        return obstacleCourse.findPathSegmentsToPoint(startTime=0.0,
                                                      startPoint=self._position,
                                                      startSpeed=self._speed,
                                                      startUnitVelocity=self._direction,
                                                      targetPoint=self._waypoint._position,
                                                      velocityOfTarget=_zero,
                                                      legalRotDirection=0.0)

    def skirtingPathSegments(self, obstacleCourse):
        return obstacleCourse.findPathSegments(startTime=0.0,
                                                      startPoint=self._position,
                                                      startSpeed=self._speed,
                                                      startUnitVelocity=self._direction,
                                                      legalRotDirection=0.0)

    def drawPath(self, canvas, **kwargs):
        pass

    def draw(self, canvas, **kwargs):
        gui.draw.drawPoint(canvas, self._position, **kwargs)


class Vertex(BaseVertex):

    def __init__(self, waypoint, priorityHeuristic, admissibleHeuristic, previousVertex=None,
                 pathSegment=None):
        BaseVertex.__init__(self, waypoint)

        # The timeToVertex from start to this vertex
        self.timeToVertex = previousVertex.getTimeTo() + pathSegment.elapsedTime
        
        # Estimates of the timeToVertex required to traverse the path from start to finish through this point.

        # Priority weights unknown heuristic portion higher, to prioritize vertices closer to the end
        self.timeEstimatePriority = self.timeToVertex + priorityHeuristic
        
        # "Admissible" estimate is a best case time, which may not be very realistic.  
        # A vertex is not thrown out unless the best known path is better than this
        self.timeEstimateAdmissible = self.timeToVertex + admissibleHeuristic
        
        # The previous vertex which is part of the shortest path from start through this vertex
        self.previousVertex = previousVertex

        # A path segment from the previous vertex to this vertex.  Stored for rendering/debug.
        self.pathSegment = pathSegment

        # Potentially holds information for debugging
        self.debug = None

    def getPosition(self):
        return self.pathSegment.endPoint

    def getVelocity(self):
        return self.pathSegment.endUnitVelocity * self.pathSegment.endSpeed

    def getTimeTo(self):
        return self.timeToVertex

    def getTimeThroughAdmissible(self):
        return self.timeEstimateAdmissible

    def getTimeThroughPriority(self):
        return self.timeEstimatePriority

    def getPreviousVertex(self):
        return self.previousVertex
    
    def pathSegmentsToWaypoint(self, obstacleCourse):
        return obstacleCourse.findPathSegmentsToPoint(startTime=self.timeToVertex,
                                                      startPoint=self.pathSegment.endPoint,
                                                      startSpeed=self.pathSegment.speed,
                                                      startUnitVelocity=self.pathSegment.endUnitVelocity,
                                                      targetPoint=self._waypoint._position,
                                                      velocityOfTarget=_zero,
                                                      legalRotDirection=self.pathSegment.nextLegalRotDirection)

    def skirtingPathSegments(self, obstacleCourse):
        return obstacleCourse.findPathSegments(startTime=self.timeToVertex,
                                                      startPoint=self.pathSegment.endPoint,
                                                      startSpeed=self.pathSegment.speed,
                                                      startUnitVelocity=self.pathSegment.endUnitVelocity,
                                                      legalRotDirection=self.pathSegment.nextLegalRotDirection)

    def drawPath(self, canvas, **kwargs):
        if self.previousVertex is not None:
            self.pathSegment.draw(canvas, **kwargs)
            self.previousVertex.drawPath(canvas, **kwargs)

    def draw(self, canvas, **kwargs):
        gui.draw.drawPoint(canvas, self._position, **kwargs)
        gui.draw.drawText(canvas, (self._position[0] + TEXT_OFFSET, self._position[1]),
                          text="{:4.2f}".format(self.timeToVertex), **kwargs)

    def __str__(self):
        return "(" + str(self._position[0]) + "," + str(self._position[1]) + ") with cost: " + str(self.timeToVertex)

