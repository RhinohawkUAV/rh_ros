from threading import Thread, Condition
import threading
from engine.pathFinder import PathFinder


class PathFinderManager:
    """
    Manages path finding tasks in a background thread.  Provides the following abilities:
    1. Submit a new path finding task.  This will cancel and previous/outstanding calculations.
    2. Request that one or more steps be performed.  Steps will stop being performed if solved.
    3. Request that the problem be solved or a timeout occurs.
    4. Shutdown the manager.
    """

    def __init__(self):
        self._controlLock = Condition(threading.Lock())
        self._steps = 0
        self._shutdown = False
        self._activePathFinder = None
        self._thread = Thread(target=self._run)
        self._thread.start()

    def publishSolution(self, solutionPathSegments, finished):
        """
        Override me.
        Called whenever a step on the active path finder concludes with a solutionPathSegments.
        This is called from within the path finder thread and should execute quickly.
        """
        pass
    
    def publishDebug(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        """
        Override me.
        Called whenever a step on the active path finder concludes with debug data.
        This is called from within the path finder thread and should execute quickly.
        """
        pass
            
    def shutdown(self):
        """
        Shutdown the path finding manager and cancel all remaining steps.
        """
        with self._controlLock:
            self._shutdown = True
            self._controlLock.notifyAll()

    def shutdownAndWait(self):
        """
        Shutdown and wait for termination.
        """
        self.shutdown()
        self._thread.join()

    def submit(self, scenario, vehicle):
        """
        Submit a new path finding problem.  Will cancel any queued steps.  
        If a step is currently executing, its result will not be published.
        """
        with self._controlLock:
            self._activePathFinder = PathFinder(scenario, vehicle)
            self._steps = 0

    def step(self, numSteps=1):
        """
        Request the path finder run until:
        1. the given number of steps are performed
        2. A solution is found
        3. A new problem is submitted
        """
        with self._controlLock:
            if self._activePathFinder is None or self._activePathFinder.isDone():
                return
            self._steps += numSteps
            self._controlLock.notifyAll()

    def solve(self, timeout):
        # TODO: Implement
        pass

    def _run(self):
        try:
            while True:
                pathFinder = self._getNextStep()
                self._performStep(pathFinder)
        except ShutdownException:
            pass

    def _checkShutdown(self):
        if self._shutdown:
            raise ShutdownException            

    def _getNextStep(self):
        """
        Waits until there is a step to perform and returns the path finder to perform it on.  
        May throw a ShutdownException instead if ROS is shutting down
        """
        with self._controlLock:
            self._checkShutdown()
            while self._steps == 0:
                self._controlLock.wait()
                self._checkShutdown()

            return self._activePathFinder

    def _performStep(self, pathFinder):
        """
        Runs a step on the given path finder instance.  Afterwards one of 3 things can happen:
        1. Can throw a ShutdownException if manager was shutdown.
        2. Can do nothing if the path finder, the step was performed on, is no longer active (a new problem submitted)
        3. Can publish result through overrideable publishing methods.
        """
        # Calculate a step for the given path finder.  This takes non-zero time and is therefore not syncrhonized.
        solutionFound = pathFinder.step()
        
        with self._controlLock:
            # If ROS was shutdown while not locked, then throw exception and exit
            self._checkShutdown()
            # If the active path finder has changed, then do not publish.
            if pathFinder is not self._activePathFinder:
                return
            
            # Publish and decrement number of steps
            if solutionFound:
                self.publishSolution(pathFinder.getSolution(), pathFinder.isDone())
            else:
                debugData = pathFinder.getDebugData()
                self.publishDebug(*debugData)
            self._steps -= 1


class ShutdownException(BaseException):
    pass
