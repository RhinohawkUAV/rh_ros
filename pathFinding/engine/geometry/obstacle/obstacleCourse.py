from engine.geometry.obstacle.arcFinder.arcSegmentFinder import ArcSegmentFinder
from engine.geometry.obstacle.intersectionDetector.cPathIntersectionDetector import CPathIntersectionDetector
from engine.geometry.obstacle.intersectionDetector.pyPathIntersectionDetector import PyPathIntersectionDetector
from gui.core import Drawable


def createObstacleCourse(params, vehicle, scenario):
    pathSegmentFinder = ArcSegmentFinder(params, vehicle)
    pathSegmentFinder.setState(scenario.boundaryPoints, scenario.noFlyZones, scenario.dynamicNoFlyZones)    
#   pathSegmentFinder = LineSegmentFinder(params, vehicle)
#     pathIntersectionDetector = PyPathIntersectionDetector(params.nfzBufferWidth, scenario.boundaryPoints, scenario.noFlyZones, scenario.dynamicNoFlyZones)
    pathIntersectionDetector = CPathIntersectionDetector(params.nfzBufferWidth, scenario.boundaryPoints, scenario.noFlyZones, scenario.dynamicNoFlyZones)
    return ObstacleCourse(pathSegmentFinder, pathIntersectionDetector)

    
class ObstacleCourse(Drawable):

    def __init__(self, pathSegmentFinder, pathIntersectionDetector):
        self.pathSegmentFinder = pathSegmentFinder
        self.pathIntersectionDetector = pathIntersectionDetector

    def stall(self, startTime, startPoint, startSpeed, startUnitVelocity, minStallTime, ignoreBuffers=False):
        """
        Find legal path segments for use in a stall manuever.  Stall is a way to waste time, for wasting time's sake!  This
        allows exploration of inefficient paths, which may delay while obstacles move around and windows open.
        :param minStallTime: The minimum amount of time to waste.  Typically its not valuable to stall for a very short 
        period of time as the path finder will naturally be able to find solutions which are slightly slower if necessary.
        """
        pathSegments = self.pathSegmentFinder.stall(startTime,
                                                    startPoint,
                                                    startSpeed,
                                                    startUnitVelocity,
                                                    minStallTime)
        return self._filterPathSegments(pathSegments, ignoreBuffers=ignoreBuffers)
        
    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget, legalRotDirection, ignoreBuffers=False):
        """
        Find legal path segments from a given starting point and velocity to the moving target, ending at finalSpeed.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.

        :param startTime: the time at which the path starts
        :param startPoint: where the path starts
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :param targetPoint: the destination point
        :param velocityOfTarget: the velocity of the destination (this is usually a point on a NFZ, which may be moving)
        :return: (valid,filtered)  The 1st list of valid path segments (do not intersect NFZs).  The 2nd list 
        shows path segments that were filtered, for debugging purposes.
        """
        pathSegments = self.pathSegmentFinder.findPathSegmentsToPoint(startTime,
                                                                      startPoint,
                                                                      startSpeed,
                                                                      startUnitVelocity,
                                                                      targetPoint,
                                                                      velocityOfTarget,
                                                                      legalRotDirection)
        (valid, filtered) = self._filterPathSegments(pathSegments, ignoreBuffers=ignoreBuffers)
        self.pathSegmentFinder.sortPathSegments(valid)
        return (valid, filtered)

    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection, ignoreBuffers=False):
        """
        Find legal path segments from a given starting point and velocity to vertices of no fly zones.
        This takes into account the time at which this query is made, which will affect the position of DNFZs.
        :param startTime: the time at which the path starts
        :param startPoint:
        :param startSpeed: the speed of the vehicle at the start of the path
        :param startUnitVelocity: the direction of the vehicle at the start of the path
        :return: (valid,filtered)  The 1st list of valid path segments (do not intersect NFZs).  The 2nd list 
        shows path segments that were filtered, for debugging purposes.
        """
        pathSegments = self.pathSegmentFinder.findPathSegments(startTime, startPoint, startSpeed, startUnitVelocity, legalRotDirection)
        (valid, filtered) = self._filterPathSegments(pathSegments, ignoreBuffers=ignoreBuffers)
        self.pathSegmentFinder.sortPathSegments(valid)
        return (valid, filtered)

    def _filterPathSegments(self, pathSegments, ignoreBuffers):
        unfilteredPathSegments = []
        filteredPathSegments = []
        for pathSegment in pathSegments:
            if pathSegment.testIntersection(self.pathIntersectionDetector):
                filteredPathSegments.append(pathSegment)
            else:
                unfilteredPathSegments.append(pathSegment)
        return (unfilteredPathSegments, filteredPathSegments)   

    def draw(self, canvas, time=0.0, **kwargs):
        self.pathSegmentFinder.draw(canvas, time=time, **kwargs)
        self.pathIntersectionDetector.draw(canvas, time=time, **kwargs)
        
