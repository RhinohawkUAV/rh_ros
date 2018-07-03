from constants import DEFAULT_MAX_VEHICLE_SPEED, DEFAULT_TURN_ACCELERATION


class Vehicle:

    def __init__(self, maxSpeed, acceleration):
        self.maxSpeed = maxSpeed
        self.acceleration = acceleration


DEFAULT_VEHICLE = Vehicle(DEFAULT_MAX_VEHICLE_SPEED, DEFAULT_TURN_ACCELERATION)
