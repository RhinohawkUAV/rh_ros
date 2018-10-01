from threading import Thread
import time

from engine.pathFinder import PathFinder
from engine.pathFinderManager import PathFinderManager
import gui
from gui.pathFinder.pathFinderInterface import PathFinderInterface


class LocalPathFinderInterface(PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self):
        self._pathFinderManager = PathFinderManager()
        self._pathFinderManager.setListeners(self._fireInputAccepted, self._fireStepPerformed)
        self._solving = False

    def submitProblem(self, params, scenario, vehicle):
        
        if self._solving:
            print "Cannot submit problem while other operations are in progress!"
            return 
        self._pathFinderManager.submitProblem(params, scenario, vehicle)
    
    def stepProblem(self, numSteps=1):
        if self._solving:
            print "Cannot step problem while other operations are in progress!"
            return 
        
        self._pathFinderManager.stepProblem(numSteps)
        
    def solveProblem(self, params, scenario, vehicle, timeout):
        startTime = time.time()
        if self._pathFinderManager.getStepsRemaining() > 0 or self._solving:
            print "Cannot start a solve operation while other operations are in progress!"
            return 
            self._solving = True
        
        Thread(target=self._solve, args=[params, scenario, vehicle, startTime, timeout]).start()

    def _solve(self, params, scenario, vehicle, startTime, timeout):
        self._fireInputAccepted(params, scenario, vehicle)
        pathFinder = PathFinder(params, scenario, vehicle)
        totalTime = 0.0
        numSteps = 0
        
        # Timeout occurs when total_time + avg_time_per_step > timeout
        while pathFinder.step():
            totalTime = time.time() - startTime
            numSteps += 1
            if (totalTime * (numSteps + 1)) / numSteps > timeout:
                break
        bestPath = pathFinder.getBestPath()
        self._fireSolved(bestPath)
        gui.inGUIThread(self._endSolve)

    def _endSolve(self):
        self._solving = False
