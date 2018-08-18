class ObstacleCourse:

    def __init__(self, pathSegmentFinder, pathIntersectionDetector):
        self.pathSegmentFinder = pathSegmentFinder
        self.pathIntersectionDetector = pathIntersectionDetector

    def setInitialState(self, boundaryPoints, noFlyZones):
        """
        Set the state of the static obstacle data.
        :param boundaryPoints:
        :param noFlyZones:
        :return:
        """
        self.pathSegmentFinder.setInitialState(boundaryPoints, noFlyZones)
        self.pathIntersectionDetector.setInitialState(boundaryPoints, noFlyZones)
       
    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        """
        Set the state of the dynamic obstacle data at time=0.0.
        :param dynamicNoFlyZones:
        :return:
        """
        
        self.dynamicNoFlyZones = dynamicNoFlyZones
        self.pathSegmentFinder.setDynamicNoFlyZones(dynamicNoFlyZones)
        self.pathIntersectionDetector.setDynamicNoFlyZones(dynamicNoFlyZones)

    def findPathSegmentsToPoint(self, startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget):
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
        pathSegments = self.pathSegmentFinder.findPathSegmentsToPoint(startTime, startPoint, startSpeed, startUnitVelocity, targetPoint, velocityOfTarget)
        return self._filterPathSegments(pathSegments)

    def findPathSegments(self, startTime, startPoint, startSpeed, startUnitVelocity):
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
        pathSegments = self.pathSegmentFinder.findPathSegments(startTime, startPoint, startSpeed, startUnitVelocity)
        return self._filterPathSegments(pathSegments)
    
    def _filterPathSegments(self, pathSegments):
        unfilteredPathSegments = []
        filteredPathSegments = []
        for pathSegment in pathSegments:
            if pathSegment.testIntersection(self.pathIntersectionDetector):
                filteredPathSegments.append(pathSegment)
            else:
                unfilteredPathSegments.append(pathSegment)
        return (unfilteredPathSegments, filteredPathSegments)   
