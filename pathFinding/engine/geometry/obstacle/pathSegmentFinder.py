import math

from engine.geometry import calcs
from engine.geometry.obstacle.vertexTarget import VertexTarget
from utils import profile


class PathSegmentFinder(object):

    def setInitialState(self, boundaryPoints, noFlyZones):
        """
        Set the state of the static obstacle data.
        :param boundaryPoints:
        :param noFlyZones:
        :return:
        """
        for nfz in noFlyZones:
            self.createNFZTargets(nfz)
    
    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        """
        Set the state of the dynamic obstacle data at time=0.0.
        :param dynamicNoFlyZones:
        :return:
        """
        pass 

    def createNFZTargets(self, nfz):
        for i in range(-1, len(nfz.points) - 1):
            normal = calcs.CCWNorm(nfz.points[i + 1] - nfz.points[i - 1])
            normal = calcs.unit(normal)
            
            # TODO: NFZs are wound CW (we should make CCW given other conventions)
            pointAngle = calcs.calcVertexAngle(nfz.points[i + 1], nfz.points[i], nfz.points[i - 1])
            if pointAngle < math.pi:
                self.createVertexTarget(nfz.points[i], nfz.velocity, normal, pointAngle)

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

