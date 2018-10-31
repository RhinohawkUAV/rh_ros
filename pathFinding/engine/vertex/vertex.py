import copy

from gui import Drawable
import gui.draw
import numpy as np

DRAW_RADIUS = 0.5
_zero = np.array((0, 0), np.double)


class BaseVertex(Drawable):
    """
    Represents a potentially reachable state.  This is defined by:
    elapsed time
    position
    velocity
    next waypoint
    
    Vertices form a tree branching from the start position outwards.  Unlike many traditional graph searches, this is NOT a DAG.
    The reason is that it is incredibly unlikely that two branches will ever lead to the same conditions (position, velocity, time, etc).
    If any of these quantities is even slightly different they might not be able to reach the same solution set.
    
    NOTE:
    A future version of this could allow for similar vertices to be "merged".  Merging will result in loss of "resolution" in the solution.
    This means that it is possible that solutions through small gaps, or requiring exact timing, may not be found.  This could be OK as we 
    already put buffers around NFZs, which has the same effect.  Not clear how much the "merge radius" would affect the resolution and if
    any significant speed benefit would result from the price we would be willing to pay in terms of resolution.
    """

    def __init__(self, _nextWaypoint):
        self._nextWaypoint = _nextWaypoint

    def isSolution(self):
        """
        Vertex is end point of a completed solution path.
        """
        return self._nextWaypoint is None
    
    def getNextWaypoint(self):
        """
        The next waypoint the vehicle is seeking.  Returns None, if this vertex is an end point.
        """
        return self._nextWaypoint

    def getPosition(self):
        """
        The position of the vehicle.
        """
        pass

    def getVelocity(self):
        """
        The velocity vector, of the vehicle, at this vertex.
        """
        pass

    def getTimeTo(self):
        """
        The time required to get to this vertex from the start.
        """
        pass
    
    def getTimeThroughAdmissible(self):
        """
        An optimistic estimated total time through this vertex, to the end.
        This is more or less admissible meaning that this estimate is
        unlikely to be worse than reality.
        """
        pass

    def getTimeThroughHeuristic(self):
        """
        An estimated total time through this vertex, to the end.  This is
        less optimistic by applying a multiplier (vertexHeuristicMultiplier) 
        to the admissible (optimistic) estimated remaining time.
        """
        pass

    def getPreviousVertex(self):
        """
        The previous vertex in this path.
        """
        pass

    def pathSegmentsToWaypoint(self, obstacleCourse):
        """
        Query obstacle course to get a list of path segments from this vertex to the next waypoint.
        """
        pass

    def skirtingPathSegments(self, obstacleCourse):
        """
        Query obstacle course to get a list of path segments from this vertex which skirt near obstacle boundaries.
        """
        pass

    def drawPath(self, visualizer, **kwargs):
        pass

    def draw(self, visualizer, **kwargs):
        pass

    def generatePathSegments(self):
        pathSegments = []
        currentVertex = self
        previousVertex = currentVertex.getPreviousVertex()
    
        while not previousVertex is None:
            pathSegments.append(copy.deepcopy(currentVertex.pathSegment))
            currentVertex = previousVertex
            previousVertex = currentVertex.getPreviousVertex()
        
        # We traced the path backwards, so reverse
        pathSegments.reverse()
    
        return pathSegments

    
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
        """
        Considered the worst possible vertex, this forces it to be replaced as the best path immediately.
        """
        return float("inf")

    def getTimeThroughHeuristic(self):
        """
        Considered the worst possible vertex, this forces it to be replaced as the best path immediately.
        """
        return float("inf")
    
    def getPreviousVertex(self):
        return None

    def stall(self, obstacleCourse, minStallTime):
        return obstacleCourse.stall(startTime=0.0,
                                    startPoint=self._position,
                                    startSpeed=self._speed,
                                    startUnitVelocity=self._direction,
                                    minStallTime=minStallTime)
        
    def pathSegmentsToWaypoint(self, obstacleCourse): 
        return obstacleCourse.findPathSegmentsToPoint(startTime=0.0,
                                                      startPoint=self._position,
                                                      startSpeed=self._speed,
                                                      startUnitVelocity=self._direction,
                                                      targetPoint=self._nextWaypoint._position,
                                                      velocityOfTarget=_zero,
                                                      legalRotDirection=0.0)

    def skirtingPathSegments(self, obstacleCourse):
        return obstacleCourse.findPathSegments(startTime=0.0,
                                                      startPoint=self._position,
                                                      startSpeed=self._speed,
                                                      startUnitVelocity=self._direction,
                                                      legalRotDirection=0.0)

    def drawPath(self, visualizer, **kwargs):
        pass

    def draw(self, visualizer, **kwargs):
        gui.draw.drawPoint(visualizer, self._position, **kwargs)


class Vertex(BaseVertex):

    def __init__(self, waypoint, priorityHeuristic, admissibleHeuristic, previousVertex=None,
                 pathSegment=None):
        BaseVertex.__init__(self, waypoint)

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

    def getTimeThroughHeuristic(self):
        return self.timeEstimatePriority

    def getPreviousVertex(self):
        return self.previousVertex
    
    def stall(self, obstacleCourse, minStallTime):
        return obstacleCourse.stall(startTime=self.timeToVertex,
                                    startPoint=self.pathSegment.endPoint,
                                    startSpeed=self.pathSegment.endSpeed,
                                    startUnitVelocity=self.pathSegment.endUnitVelocity,
                                    minStallTime=minStallTime)
    
    def pathSegmentsToWaypoint(self, obstacleCourse):
        return obstacleCourse.findPathSegmentsToPoint(startTime=self.timeToVertex,
                                                      startPoint=self.pathSegment.endPoint,
                                                      startSpeed=self.pathSegment.endSpeed,
                                                      startUnitVelocity=self.pathSegment.endUnitVelocity,
                                                      targetPoint=self._nextWaypoint._position,
                                                      velocityOfTarget=_zero,
                                                      legalRotDirection=self.pathSegment.nextLegalRotDirection)

    def skirtingPathSegments(self, obstacleCourse):
        return obstacleCourse.findPathSegments(startTime=self.timeToVertex,
                                                      startPoint=self.pathSegment.endPoint,
                                                      startSpeed=self.pathSegment.endSpeed,
                                                      startUnitVelocity=self.pathSegment.endUnitVelocity,
                                                      legalRotDirection=self.pathSegment.nextLegalRotDirection)

    def drawPath(self, visualizer, **kwargs):
        if self.previousVertex is not None:
            self.pathSegment.draw(visualizer, **kwargs)
            self.previousVertex.drawPath(visualizer, **kwargs)

    def draw(self, visualizer, **kwargs):
        gui.draw.drawPoint(visualizer, self._position, **kwargs)
        gui.draw.drawText(visualizer, (self._position[0], self._position[1]),
                          text="{:4.2f}".format(self.timeToVertex), offsetY=10.0, **kwargs)

    def __str__(self):
        return "(" + str(self._position[0]) + "," + str(self._position[1]) + ") with cost: " + str(self.timeToVertex)

