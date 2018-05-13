import gui


class PathFinderListener(object):
    """
    Anything registered to listen to the pathFinderInterface.  All signals will occur on the GUI thread.
    """
    
    def debug(self, pathSegments, filteredSegments, currentPath):
        pass

    def solution(self, solutionPath):
        pass

    def triggerDebug(self, pathSegments, filteredSegments, currentPath):
        """
        For convenience of the pathFinderInterface.  This will call debug() in the GUI thread with given arguments.
        """
        gui.inGUIThread(self.debug, pathSegments, filteredSegments)

    def triggerSolution(self, solutionPath):
        """
        For convenience of the pathFinderInterface.  This will call debug() in the GUI thread with given arguments.
        """
        gui.inGUIThread(self.solution, solutionPath)
