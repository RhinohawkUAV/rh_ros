from constants import NFZ_MAX_NEW_VERTEX_EXPANSION_RATIO
from engine.geometry import calcs
from gui.core import Drawable
import numpy as np


class PathSegmentFinder(Drawable):
    
    def __init__(self, targetOffset):
        self.targetOffset = targetOffset
        
    def setInitialState(self, boundaryPoints, noFlyZones):
        """
        Set the state of the static obstacle data.
        :param boundaryPoints:
        :param noFlyZones:
        :return:
        """
        for nfz in noFlyZones:
            self.createPolygonTargets(nfz.points, nfz.velocity)

        self.createPolygonTargets(boundaryPoints, np.array((0.0, 0.0), np.double))
        
    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        """
        Set the state of the dynamic obstacle data at time=0.0.
        :param dynamicNoFlyZones:
        :return:
        """
        pass 

    def createPolygonTargets(self, points, velocity):
        points = calcs.calcShell(points, self.targetOffset)
        for i in range(-1, len(points) - 1):
            pointAngle = calcs.calcVertexAngle(points[i - 1], points[i], points[i + 1])
            pointNormal = calcs.calcVertexNormal(points[i - 1], points[i], points[i + 1])
            # TODO: Should not create target if angle will not accept any incoming velocities!!
            self.createVertexTarget(points[i], velocity, pointNormal, pointAngle)

    def createVertexTarget(self, point, velocity, normal, pointAngle):
        pass

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
        """
        Find path segments from a given starting point and velocity to the moving target.  Does not account for interesections with other
        NFZs or the boundary.  This takes into account the time at which this query is made, which will affect the position of DNFZs.

        :param startTime: the time at which the path starts
        :param startPoint: where the path starts
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param targetPoint: the destination point
        :param velocityOfTarget: the velocity of the destination (this is usually a point on a NFZ, which may be moving)
        :return: a list of PathSegments describing routes to the given point
        """
        pass
   
    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
        """
        Find path segments from a given starting point and velocity to skirting postions near no fly zones and boundary points.  
        Does not account for interesections with other NFZs or the boundary.  This takes into account the time at which this 
        query is made, which will affect the position of DNFZs.
        :param startTime: the time at which the path starts
        :param startPoint:
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :return: a list of PathSegments describing routes from the start point to skirting points around no fly zones
        """
        pass

