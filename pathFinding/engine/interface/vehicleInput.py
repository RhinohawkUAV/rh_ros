from constants import DEFAULT_MAX_VEHICLE_SPEED, DEFAULT_TURN_ACCELERATION


class VehicleInput:

    def __init__(self, maxSpeed, acceleration):
        self.maxSpeed = maxSpeed
        self.acceleration = acceleration


DEFAULT_VEHICLE = VehicleInput(DEFAULT_MAX_VEHICLE_SPEED, DEFAULT_TURN_ACCELERATION)
