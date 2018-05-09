import numpy as np
from engine.geometry import calcs

class ScenarioInput:
    def __init__(self, boundaryPoints, noFlyZones, roads, startPoint, startVelocity, wayPoints):
        # Defines the boundary polygon (geo-fence) for the path-finding problem.  According to the rules this can be
        # max 18 sided, this will accept any number of sides.
        # Enforce CW winding (normals face inward)

        self.boundaryPoints = np.array(boundaryPoints, np.double)
        if not calcs.woundCCW(self.boundaryPoints):
            self.boundaryPoints = np.flipud(self.boundaryPoints)
            # TODO: Test that this really flips properly

        # A sequence of NoFlyZoneInput objects.  Not clear if rules allow dynamic NFZs can be announced initially,
        # but this allows for that case.
        self.noFlyZones = noFlyZones
        
        self.roads = roads
        
        self.startPoint = np.array(startPoint, np.double)
        self.startVelocity = np.array(startVelocity, np.double)
        self.wayPoints = np.array(wayPoints, np.double)
        

    def calcBounds(self):
        return calcs.calcBounds(self.boundaryPoints)

