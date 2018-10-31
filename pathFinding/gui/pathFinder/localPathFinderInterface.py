import errno
import os
from threading import Thread
import time

from engine.interface import fileUtils
from engine.pathFinder import PathFinder
from engine.pathFinderManager import PathFinderManager
import gui
from gui.pathFinder.pathFinderInterface import PathFinderInterface

MAX_FILE_DUMPS = 1000


class LocalPathFinderInterface(PathFinderInterface):
    """
    Manages a local path finder for use in PathFindViewer.
    """

    def __init__(self, dumpScenarios=False):
        PathFinderInterface.__init__(self)
        self._pathFinderManager = PathFinderManager()
        self._pathFinderManager.setListeners(self._fireInputAccepted, self._fireStepPerformed)
        self._solving = False
        self._solveID = 0
        self._dumpScenarios = dumpScenarios

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
            
        if self._dumpScenarios:
            self._solveID += 1
            solveID = self._solveID % MAX_FILE_DUMPS
            dumpDirectory = os.path.join(os.path.expanduser("~"), "pathDumps")
            if not os.path.exists(dumpDirectory):
                try:
                    os.makedirs(dumpDirectory)
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise
            path = os.path.join(dumpDirectory, "dump" + str(solveID) + ".json")
            print "Solving problem ID = " + str(solveID) + ".  Problem stored in: " + path
            fileUtils.save(path, params, scenario, vehicle)     
                    
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
        pathFinder.destroy()
        self._solving = False
        self._fireSolved(bestPath)
