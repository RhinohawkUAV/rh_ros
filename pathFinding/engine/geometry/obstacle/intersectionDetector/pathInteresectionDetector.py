class PathIntersectionDetector:

    def setInitialState(self, boundaryPoints, noFlyZones):
        """
        Set the state of the static obstacle data.
        :param boundaryPoints:
        :param noFlyZones:
        :return:
        """
        pass

    def setDynamicNoFlyZones(self, dynamicNoFlyZones):
        """
        Set the state of the dynamic obstacle data at time=0.0.
        :param dynamicNoFlyZones:
        :return:
        """
        pass
    
    def testIntersections(self, lineSets):
        pass
    
    def testStraightPathInteresection(self, startTime, startPoint, endPoint, speed):
        pass
