from engine.geometry import calcs


class Target:

    def __init__(self, startPosition, velocity):
        self._startPosition = startPosition
        self.position = None
        self.velocity = velocity
        (self.direction, self.speed) = calcs.unitAndLength(self.velocity)
    
    def update(self, time):
        """
        Updates position based on the current time.  Should be called on a target for each time at which computation will take place.
        """
        self.position = self._startPosition + self.velocity * time

    def getPosition(self, time):
        """
        Starting at position (not _startPosition) get a relative, future position at the given time.
        """
        return self.position + self.velocity * time
