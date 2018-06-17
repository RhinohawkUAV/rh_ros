# Based on the paper Datum Transformations of GPS Positions (https://www.u-blox.com/en)
import math
from numpy.random.mtrand import np
from pathfinding.msg._Vec2 import Vec2

import constants
from engine.geometry import calcs

f = 1.0 / 298.257223563
a = 6378137.0
b = a * (1 - f)
e1 = math.sqrt((a * a - b * b) / (a * a))
e2 = math.sqrt((a * a - b * b) / (b * b))
bSq_aSq = (b * b) / (a * a)


def _radiusCurvature(latRad):
    val = e1 * math.sin(latRad)
    return a / math.sqrt(1 - val * val)


northPoleECEF = np.array((0, 0, bSq_aSq * _radiusCurvature(math.pi / 2) * math.sin(math.pi / 2)), np.double)


class GPSTransformer:

    def __init__(self, referenceGPS):
        self.localZECEF = self._gpsToECEF(referenceGPS)
        localUnitZECEF = self.localZECEF / np.linalg.norm(self.localZECEF)
        unitToPole = calcs.unit(northPoleECEF - self.localZECEF)
        self.localUnitYECEF = unitToPole - localUnitZECEF * np.dot(unitToPole, localUnitZECEF)
        self.localUnitYECEF = calcs.unit(self.localUnitYECEF)
        self.localUnitXECEF = np.cross(self.localUnitYECEF, localUnitZECEF)
        self.localUnitZECEF = calcs.unit(self.localZECEF)
    
    def gpsToLocal(self, gps):
        """
        Converts a GPS Vec2 to a 2d np.Array in local coordinate space
        gps.x = longitude
        gps.y = latitude
        Altitude is ignored and assumed to be 0 for all points.
        """        
        ecef = self._gpsToECEF(gps)
        return self._ecefToLocal(ecef)

    def localToGPS(self, local):
        """
        Converts a 2d np.Array in local coordinate space to a GPS Vec2
        gps.x = longitude
        gps.y = latitude
        Altitude is ignored and assumed to be 0 for all points.
        """        
        ecef = self._localToECEF(local)
        return self._ecefToGPS(ecef)

    def gpsVelocityToLocal(self, gpsVelocity):
        """
        Converts a gps velocity to a 2D np.Array in local coordinates.
        @param gpsVelocity Vec2(speed,angle), speed in m/2, angle = 0 for north and angle = 90 for east
        """
        speed = gpsVelocity.x
        angle = math.radians(gpsVelocity.y)
        return np.array((speed * math.sin(angle), speed * math.cos(angle)), np.double)
    
    def localToGPSVelocity(self, local):
        """
        2D np.Array in local coordinates to a GPS velocity
        A GPS velocity is Vec2(speed,angle), speed in m/2, angle = 0 for north and angle = 90 for east
        """
        speed = math.sqrt(local[0] * local[0] + local[1] * local[1])
        angle = math.degrees(math.atan2(local[0], local[1]))
        return Vec2(speed, angle)

    def _gpsToECEF(self, gps):
        """
        Converts a GPS Vec2 to a (X,Y,Z) ECEF np.Array.
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
        """
        Converts  ECEF np.Array to a 2d np.Array in local coordinate space.
        """
        ecef -= self.localZECEF
        return np.array((np.dot(self.localUnitXECEF, ecef), np.dot(self.localUnitYECEF, ecef)), np.double)
    
    def _ecefToGPS(self, ecef):
        """
        Converts an ECEF np.Array to GPS Vec2 (longitude,lattitude).
        """
        longitude = math.atan2(ecef[1], ecef[0])
        
        p = math.sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1])
        theta = math.atan2(ecef[2] * a, p * b)
        cosTheta = math.cos(theta)
        sinTheta = math.sin(theta)
        latY = ecef[2] + e2 * e2 * b * sinTheta * sinTheta * sinTheta
        latX = p - e1 * e1 * a * cosTheta * cosTheta * cosTheta
        lat = math.atan2(latY, latX)
        return Vec2(math.degrees(longitude), math.degrees(lat))
    
    def _localToECEF(self, localVec2):
        """
        Converts a 2d np.Array in local coordinate space to an ECEF np.Array.
        """
        return localVec2[0] * self.localUnitXECEF + localVec2[1] * self.localUnitYECEF + self.localZECEF


if __name__ == "__main__":
    # Canberra
    gpsRef = Vec2(constants.CANBERRA_GPS[0], constants.CANBERRA_GPS[1])
    trans = GPSTransformer(gpsRef)
    
    # A good worst case would be if start an end were at diagonals of a square 
    # and the path were a completely straight line.  If the boundaries were just barely surrounding
    # this square, then they would be ~30km appart (16 nautical miles).
    # If we set the reference position to be the center of this square then
    # the furthest corner would be able (15km/6371km) radians away.
    maxAngleDiff = math.degrees(15.0 / 6371.0)
    
    # In this example we pick a point 
    # which is max angle away in lattitude AND longitude, which is worse than the worst case!
    gps = Vec2(gpsRef.x - maxAngleDiff, gpsRef.y - maxAngleDiff)
    gps2 = trans.localToGPS(trans.gpsToLocal(gps))
    
    diff = Vec2(math.radians(gps2.x - gps.x), math.radians(gps2.y - gps.y))
    
    # Norm together the two angles (more or less correct)
    angleError = math.acos(math.cos(diff.x) * math.cos(diff.y))
    
    # Distance error
    errorMeters = angleError * 6371000.0
    
    # 0.465086077192m - This is not significant so we are all good!
    print errorMeters
