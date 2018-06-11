# Based on the paper Datum Transformations of GPS Positions (https://www.u-blox.com/en)
import math
from numpy.random.mtrand import np
from pathfinding.msg._Vec2 import Vec2

f = 298.257223563
a = 6378137.0
b = a * (1 - f)
e1 = math.sqrt((a * a - b * b) / (a * a))
e2 = math.sqrt((a * a - b * b) / (b * b))
bSq_aSq = (b * b) / (a * a)


def _radiusCurvature(latRad):
    val = e1 * math.sin(latRad)
    return a / math.sqrt(1 - val * val)


northPoleECEF = np.array((0, 0, bSq_aSq * _radiusCurvature(math.pi / 2) * math.sin(math.pi / 2)), np.double)


class Transformer:

    def __init__(self, referenceGPS):
        localZECEF = self._gpsToECEF(referenceGPS)
        localUnitZECEF = localZECEF / np.linalg.norm(localZECEF)
        self.localUnitYECEF = self.localZECEF - northPoleECEF
        self.localUnitYECEF -= localZECEF * np.dot(self.localYECEF, localUnitZECEF)
        self.localUnitXECEF = self.localZECEF - northPoleECEF
    
    def _gpsToECEF(self, gps):
        """
        Converts a GPS Vec2 to an X/Y Vec2.
        gps.x = longitude
        gps.y = latitude
        Altitude is ignored and assumed to be 0 for all points.
        """
        lonRad = math.radians(gps.x)
        latRad = math.radians(gps.y)
        cosLat = math.cos(latRad)
        N = _radiusCurvature(latRad)
        x = float(N * cosLat * math.cos(lonRad))
        y = float(N * cosLat * math.sin(lonRad))
        z = float(bSq_aSq * N * math.sin(latRad))
        return np.array((x, y, z), np.double)

    def _ecefToLocal(self, ecef):
        pass
        
    def gpsToLocal(self, gps):
        ecef = self._gpsToECEF(gps)
        return self._ecefToLocal(ecef)
