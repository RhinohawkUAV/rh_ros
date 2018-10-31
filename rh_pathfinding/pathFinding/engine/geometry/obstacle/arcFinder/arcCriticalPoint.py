class ArcCriticalPoint:
    """
    Represents the point in space where a target become unreachable (inside vehicle's arc) or becomes reachable (emerges).
    Holds information about how far vehicle has to arc to get to this point and far it will have arced when the target reaches this point.
    """
 
    def __init__(self, vehicleArc, targetArc):
        
        self.vehicleArc = vehicleArc
        self.targetArc = targetArc