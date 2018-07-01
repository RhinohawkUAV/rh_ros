import gui


class PathFinderListener(object):
    """
    Anything registered to listen to the pathFinderInterface.  All signals will occur on the GUI thread.
    """
    
    def debug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        """
        Debugging information about the path solving process.  Will receive one update for every trigger of the 
        initiate/step services OR will receive continuous messages if using the solve service.
        pastPathSegments: Series of path segments, which run through the vertex currently being inspected
        futurePathSegments: List of legal path segments, forward from the vertex currently being inspected
        filteredPathSegments: List of illegal path segments, forward from the vertex currently being inspected
        """
        pass

    def solution(self, waypoints, solutionPathSegments, finished):
        """
        Called every time an improved solution is found.
        
        """
        pass

    def fireDebugInGuiThread(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        """
        For convenience of the pathFinderInterface.  This will call debug() in the GUI thread with given arguments.
        """
        gui.inGUIThread(self.debug, pastPathSegments, futurePathSegments, filteredPathSegments)

    def fireSolutionInGuiThread(self, solutionWaypoints, solutionPathSegments, finished):
        """
        For convenience of the pathFinderInterface.  This will call solution() in the GUI thread with given arguments.
        """
        gui.inGUIThread(self.solution, solutionWaypoints, solutionPathSegments, finished)

