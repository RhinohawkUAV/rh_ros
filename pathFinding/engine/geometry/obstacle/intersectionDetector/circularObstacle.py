from engine.geometry import calcs
from gui.core import Drawable
import gui.draw


class CircularObstacle(Drawable):
    
    def __init__(self, center, radius, velocity):
        self.center = center
        self.radius = radius
        self.velocity = velocity

    def checkPathIntersection(self, startTime, startPoint, velocity, pathTime):
        
        # Offset velocity by the velocity of the no-fly-zone (pretend it is not moving)
        velocity = velocity - self.velocity
        pathVector = velocity * pathTime

        # Given the start time, this dnfz will have moved.  Alternately, we offset the start and end points in the
        # opposite direction
        offset = -self.velocity * startTime
        
        return calcs.lineSegmentCircleIntersect(startPoint + offset, pathVector, self.center, self.radius)

    def draw(self, visualizer, time=0.0, **kwargs):
        gui.draw.drawCircle(visualizer, self.center + time * self.velocity, self.radius)
