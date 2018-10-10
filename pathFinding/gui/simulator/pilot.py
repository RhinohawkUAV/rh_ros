from threading import Condition, Lock, Thread


class Pilot:

    def __init__(self, pathFinderInterface, simulator):
        self._lock = Condition(Lock())
        self._simulator = simulator
        self._params = None
        self._vehicle = None
        self._shutdown = False
        self._pathFinderInterface = pathFinderInterface
        self._pathFinderInterface.addListeners(self.inputAccepted, self.stepPerformed, self.solved)
        self._thread = Thread(target=self._run)
        self._thread.start()
        
    def setState(self, params, vehicle):
        with self._lock:
            self._params = params
            self._vehicle = vehicle

    def solved(self, bestPath):
        self._simulator.setPath(bestPath)
        with self._lock:
            self._solved = True
            self._lock.notifyAll()

    def _waitForSolution(self):
        with self._lock:
            self._checkShutdown()
            while not self._solved:
                self._lock.wait()
                self._checkShutdown()

    def _getSimulator(self):
        with self._lock:
            return self._simulator

    def _run(self):
        try:
            while True:
                self._checkShutdown()
                state = self._simulator.copyState()
                if state is not None and len(state.scenario.wayPoints) > 0:
                    self._solved = False
                    self._pathFinderInterface.solveProblem(self._params, state.scenario, self._vehicle, 4.0)
                    self._waitForSolution()
                
        except self.ShutdownException:
            pass

    def shutdown(self):
        """
        Shutdown the path finding manager and cancel all remaining steps.
        """
        with self._lock:
            self._shutdown = True
            self._simulator = None

    def shutdownAndWait(self):
        """
        Shutdown and wait for termination.
        """
        self.shutdown()
        self._thread.join()

    def _checkShutdown(self):
        if self._shutdown:
            raise self.ShutdownException

    class ShutdownException(BaseException):
        pass                
    
    def inputAccepted(self, params, scenario, vehicle):
        pass
    
    def stepPerformed(self, isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments):
        pass
