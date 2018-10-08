import math

from engine.geometry import calcs
from gui.core import Drawable
import numpy as np


class PathSegmentFinder(Drawable):
    
    def __init__(self, params, vehicle):
        self.params = params
        self.vehicle = vehicle
        self.vertexTargets = []
        self.circularTargets = []
        
    def setState(self, boundaryPoints, polyNFZs, circularNFZs):
        self.vertexTargets = []
        self.circularTargets = []
        self.appendPolygonTargets(boundaryPoints, np.array((0.0, 0.0), np.double))
        for nfz in polyNFZs:
            self.appendPolygonTargets(nfz.points, nfz.velocity)
        for nfz in circularNFZs:
            self.circularTargets.append(self._createCircularTarget(nfz.center, nfz.radius, nfz.velocity))

    def _createCircularTarget(self, center, radius, velocity):
        pass

    def _createVertexTarget(self, vertexPosition, velocity, vertexNormal, vertexAngle):
        pass
        
    def appendPolygonTargets(self, points, velocity):
        points = calcs.calcShell(points, self.params.nfzTargetOffset)
        for i in range(-1, len(points) - 1):
            vertexAngle = calcs.calcVertexAngle(points[i - 1], points[i], points[i + 1])
            if vertexAngle <= math.pi:
                vertexNormal = calcs.calcVertexNormal(points[i - 1], points[i], points[i + 1])
                self.vertexTargets.append(self._createVertexTarget(points[i], velocity, vertexNormal, vertexAngle))

    # TODO: Should probably be called by PathFinder itself at some point
    def sortPathSegments(self, pathSegments):
        """
        Sort path segments in guessed order that they should be added to vertex queue.  Order matters due to uniqueness.
        """
        pass
        
    def stall(self, startTime, startPoint, startSpeed, startUnitVelocity, stallTime):
        """
        Find legal path segments for use in a stall manuever.  Stall is a way to waste time, for wasting time's sake!  This
        allows exploration of inefficient paths, which may delay while obstacles move around and windows open.
        
        :param startTime: the time at which the path starts
        :param startPoint: where the path starts
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param stallTime: The desired amount of time to waste.
        
        :return: a list of PathSegments describing routes to the given point
        """
        pass

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget, legalRotDirection):
        """
        Find path segments from a given starting point and velocity to the moving target.  Does not account for interesections with other
        NFZs or the boundary.  This takes into account the time at which this query is made, which will affect the position of DNFZs.

        :param startTime: the time at which the path starts
        :param startPoint: where the path starts
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param targetPoint: the destination point
        :param velocityOfTarget: the velocity of the destination (this is usually a point on a NFZ, which may be moving)
        :param legalRotDirection: if a previous skirting occurred
        
        :return: a list of PathSegments describing routes to the given point
        """
        pass
   
    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection):
        """
        Find path segments from a given starting point and velocity to skirting postions near no fly zones and boundary points.  
        Does not account for interesections with other NFZs or the boundary.  This takes into account the time at which this 
        query is made, which will affect the position of DNFZs.
        :param startTime: the time at which the path starts
        :param startPoint:
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param legalRotDirection: if a previous skirting occurred
        :return: a list of PathSegments describing routes from the start point to skirting points around no fly zones
        """
        pass

