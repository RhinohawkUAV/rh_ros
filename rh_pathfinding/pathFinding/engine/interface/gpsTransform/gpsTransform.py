# Based on the paper Datum Transformations of GPS Positions (https://www.u-blox.com/en)
import math
from numpy.random.mtrand import np
from pathfinding.msg._GPSCoord import GPSCoord
from pathfinding.msg._GPSVelocity import GPSVelocity

from constants import COURSE_DIM
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
        Converts a GPSCoord to a 2d np.Array in local coordinate space
        Altitude is ignored and assumed to be 0 for all points.
        """        
        ecef = self._gpsToECEF(gps)
        return self._ecefToLocal(ecef)

    def localToGPS(self, local):
        """
        Converts a 2d np.Array in local coordinate space to a GPSCoord
        Altitude is ignored and assumed to be 0 for all points.
        """        
        ecef = self._localToECEF(local)
        return self._ecefToGPS(ecef)

    def gpsVelocityToLocal(self, gpsVelocity):
        """
        Converts a gps velocity to a 2D np.Array in local coordinates.
        """
        speed = float(gpsVelocity.speed)
        angle = math.radians(float(gpsVelocity.heading))
        return np.array((speed * math.sin(angle), speed * math.cos(angle)), np.double)
    
    def localToGPSVelocity(self, local):
        """
        2D np.Array in local coordinates to a GPS velocity
        """
        speed = math.sqrt(local[0] * local[0] + local[1] * local[1])
        heading = math.degrees(math.atan2(local[0], local[1]))
        return GPSVelocity(heading, speed)

    def localAngleToGPS(self, angle, rotDirection):
        angle = math.degrees(angle) * rotDirection
        angle = angle * -1.0 + 90.0
        if angle < 0.0:
            angle += 360
        elif angle >= 360.0:
            angle -= 360.0
        return angle

    def gpsAngleToLocal(self, angle, rotDirection):
        angle = (angle - 90.0) * -1.0
        angle *= rotDirection
        if angle < 0.0:
            angle += 360
        elif angle >= 360.0:
            angle -= 360.0
        angle = math.radians(angle)
        return angle

    def _gpsToECEF(self, gps):
        """
        Converts a GPSCoord to a (X,Y,Z) ECEF np.Array.
        Altitude is ignored and assumed to be 0 for all points.
        """
        lonRad = math.radians(gps.lon)
        latRad = math.radians(gps.lat)
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
        Converts an ECEF np.Array to GPSCoord (lon,lattitude).
        """
        lon = math.atan2(ecef[1], ecef[0])
        
        p = math.sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1])
        theta = math.atan2(ecef[2] * a, p * b)
        cosTheta = math.cos(theta)
        sinTheta = math.sin(theta)
        latY = ecef[2] + e2 * e2 * b * sinTheta * sinTheta * sinTheta
        latX = p - e1 * e1 * a * cosTheta * cosTheta * cosTheta
        lat = math.atan2(latY, latX)
        return GPSCoord(math.degrees(lat), math.degrees(lon))
    
    def _localToECEF(self, local):
        """
        Converts a 2d np.Array in local coordinate space to an ECEF np.Array.
        """
        return local[0] * self.localUnitXECEF + local[1] * self.localUnitYECEF + self.localZECEF


if __name__ == "__main__":
    # Canberra
    gpsRef = GPSCoord(constants.CANBERRA_GPS[0], constants.CANBERRA_GPS[1])
    trans = GPSTransformer(gpsRef)

    # If the reference position were in the center of the square then each edge of the square would be:
    edgeAngle = math.degrees(COURSE_DIM / 2.0 / 6371000.0)
    
    # In this example we pick a point edgeAngle away in lattitude AND longitude.
    gps = GPSCoord(gpsRef.lat - edgeAngle, gpsRef.lon - edgeAngle)
    gps2 = trans.localToGPS(trans.gpsToLocal(gps))
    
    diff = GPSCoord(math.radians(gps2.lat - gps.lat), math.radians(gps2.lon - gps.lon))
    
    # Norm together the two angles (more or less correct)
    angleError = math.acos(math.cos(diff.lat) * math.cos(diff.lon))
    
    # Distance error
    errorMeters = angleError * 6371000.0

    # Error is 0 to 10 decimal places!
    print "{0:.10f}".format(errorMeters)
