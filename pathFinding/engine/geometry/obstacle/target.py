from engine.geometry import calcs


class Target:

    def __init__(self, startPosition, velocity):
        self.startPosition = startPosition
        self.position = None
        self.velocity = velocity
        (self.direction, self.speed) = calcs.unitAndLength(self.velocity)
    
    def getPosition(self, time):
        return self.startPosition + self.velocity * time
    
    def update(self, time):
        self.position = self.startPosition + self.velocity * time

