from engine.geometry import calcs
from engine.geometry.calcs import NoSolutionException


class ArcTarget:
    """
    A target the vehicle is attempting to hit/skirt with an arc+straight-line.
    """

    def initialGuess(self, arc):
        if self.notInitiallyReachable(arc):
            if self.speed == 0.0:
                # ArcTarget is unmoving and contained within the the vehicle's arc's circle.
                # It is impossible to reach this self.
                raise NoSolutionException
            criticalPoints = self.getCriticalPoints(arc)
            
            # Should be exactly 1 intersection
            if len(criticalPoints) != 1:
                raise NoSolutionException
            
            if criticalPoints[0].targetArc > criticalPoints[0].vehicleArc:
                # Vehicle reaches vehicle's arc's circle faster than target
                # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                raise NoSolutionException
            
            return criticalPoints[0].vehicleArc
            
        else:
            criticalPoints = self.getCriticalPoints(arc)
            if len(criticalPoints) > 1:
                # Path of target intersects the vehicle's arc's circle.  This requires special care in order to solve.
                if criticalPoints[0].targetArc < criticalPoints[0].vehicleArc:
                    # target falls inside vehicle's arc's circle faster than vehicle could arc there
                    
                    if criticalPoints[1].targetArc > criticalPoints[1].vehicleArc:
                        # Vehicle reaches vehicle's arc's circle faster than target
                        # Solution would require making another full loop (or multiple loops).  This is not a worthwhile target!
                        raise NoSolutionException
                    
                    return criticalPoints[1].vehicleArc
        
        arc.length = 0.0
        (angle, solution) = self.iterateSolution(arc)
        return calcs.modAngle(angle - arc.start, 0.0)

    def notInitiallyReachable(self, arc):
        """
        Is the target initially reachable?  If not then there will be only one critical point (or 0 if its not moving).  
        """
        raise NoSolutionException()

    def getCriticalPoints(self, arc):
        """
        Find the critical points for this target and the given arc.  Critical points are measaured in arc-length vehicle traveled and represent
        when the target is theoretically notInitiallyReachable by the vehicle.  
        For example when a point target falls inside the circle defining the vehicle's arc, it is no longer notInitiallyReachable.  When it reemerges then
        it could be hit again.  These are both critical points.
        """
        raise NoSolutionException()

    def iterateSolution(self, arc):
        """
        Given a vehicle's proposed arc, solve the following:
        When the end of the arc is reached, what angle would the vehicle have to travel at, in order to hit/skirt the target?
        The goal is to find an arc, whose end heading matches this angle.  The Arc finder will call this method repeatedly and
        iteratively adjust arc.length.
        
        This should return:
        (angleToHitTarget, StraightPathSolution)
        """
        raise NoSolutionException()

    def calcAvoidanceRotDirection(self, passingVelocity):
        """
        As the vehicle skirts this target, determine whether this is an avoidance in the CW (-1.0) or CCW (1.0) direction (0.0 == neither).
        """
        return 0.0
        
