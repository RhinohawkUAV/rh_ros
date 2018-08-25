from gui import Drawable
import gui.draw

DRAW_RADIUS = 0.5
TEXT_OFFSET = 3.0


class BaseVertex(Drawable):

    def getPosition(self):
        pass

    def getVelocity(self):
        pass

    def getTimeTo(self):
        pass
    
    def getTimeThrough(self):
        pass

    def getPreviousVertex(self):
        pass

    def drawPath(self, canvas, **kwargs):
        pass

    def draw(self, canvas, **kwargs):
        pass

    def pathSegmentsToPoint(self, obstacleCourse, targetPoint, velocityOfTarget):
        pass

    def skirtingPathSegments(self, obstacleCourse):
        pass

    
class OriginVertex(BaseVertex):

    def __init__(self, position, speed, direction):
        self._position = position
        self._direction = direction
        self._speed = speed

    def getPosition(self):
        return self._position

    def getVelocity(self):
        return self._direction * self._speed

    def getTimeTo(self):
        return 0.0
    
    def getTimeThrough(self):
        # Irrelevant value as this vertex will never have any competition
        return 0.0

    def getPreviousVertex(self):
        return None

    def pathSegmentsToPoint(self, obstacleCourse, targetPoint, velocityOfTarget):
        return obstacleCourse.findPathSegmentsToPoint(startTime=0.0,
                                                      startPoint=self._position,
                                                      startSpeed=self._speed,
                                                      startUnitVelocity=self._direction,
                                                      targetPoint=targetPoint,
                                                      velocityOfTarget=velocityOfTarget,
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

    def __init__(self, heuristicToGoal, previousVertex=None,
                 pathSegment=None):

        # The timeToVertex from start to this vertex
        self.timeToVertex = previousVertex.getTimeTo() + pathSegment.elapsedTime
        
        # An estimated of the timeToVertex required to traverse, the best possible path from start, through this vertex, to goal
        self.timeEstimate = self.timeToVertex + heuristicToGoal
        
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

    def getTimeThrough(self):
        return self.timeEstimate

    def getPreviousVertex(self):
        return self.previousVertex
    
    def pathSegmentsToPoint(self, obstacleCourse, targetPoint, velocityOfTarget):
        return obstacleCourse.findPathSegmentsToPoint(startTime=self.timeToVertex,
                                                      startPoint=self.pathSegment.endPoint,
                                                      startSpeed=self.pathSegment.speed,
                                                      startUnitVelocity=self.pathSegment.endUnitVelocity,
                                                      targetPoint=targetPoint,
                                                      velocityOfTarget=velocityOfTarget,
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
        gui.draw.drawPoint(canvas, self.position, **kwargs)
        gui.draw.drawText(canvas, (self.position[0] + TEXT_OFFSET, self.position[1]),
                          text="{:4.2f}".format(self.timeToVertex), **kwargs)

    def __str__(self):
        return "(" + str(self.position[0]) + "," + str(self.position[1]) + ") with cost: " + str(self.timeToVertex)
