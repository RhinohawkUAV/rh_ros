class PathFinderInterface():
    """
    Provides an interface to a path finding process for use in the GUI.  
    This allows a local path-finder or remote, ROS, path finder to use the same GUI.
    """

    def __init__(self):
        self._listener = None
    
    def setListener(self, listener):
        self._listener = listener
        
    def submitProblem(self, scenario, vehicle):
        """
        Start a new path finding process.  Will wipe out previous process.
        """
        pass
    
    def stepProblem(self, numSteps=1):
        """
        Perform one step of the path finding process.
        """
        pass