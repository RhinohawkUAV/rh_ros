from engine.geometry import calcs
import numpy as np


class Scenario:

    def __init__(self, boundaryPoints=[], noFlyZones=[], dynamicNoFlyZones=[], roads=[], startPoint=(5.0, 5.0), startVelocity=(1.0, 1.0), wayPoints=[]):
        # Defines the boundary polygon (geo-fence) for the path-finding problem.  According to the rules this can be
        # max 18 sided, this will accept any number of sides.
        self.boundaryPoints = np.array(boundaryPoints, np.double)

        # Force boundary points to be wound CW (opposite our normal convention).  
        # This will cause all normals to point inwards which is correct because we are inside.
        if calcs.woundCCW(self.boundaryPoints):
            self.boundaryPoints = np.flipud(self.boundaryPoints)

        # A sequence of Dynamic/NoFlyZoneInput objects
        self.noFlyZones = noFlyZones[:]
        self.dynamicNoFlyZones = dynamicNoFlyZones[:]
        
        self.roads = roads[:]
        
        self.startPoint = np.array(startPoint, np.double)
        self.startVelocity = np.array(startVelocity, np.double)
        self.wayPoints = np.array(wayPoints, np.double)

    def calcBounds(self):
        return calcs.calcBounds(self.boundaryPoints)
