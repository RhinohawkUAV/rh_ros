import copy
from threading import Thread, Lock, Condition

from engine.pathFinder import PathFinder


class PathFinderManager:
    """
    Manages path finding tasks in a background thread.  
    Provides the following abilities:
    1. Submit a new path finding task.  This will cancel and previous/outstanding calculations.
    2. Request that one or more steps be performed.  Steps will stop being performed if solved.
    3. Shutdown the manager.
    
    All output is reported via the given:
    inputReceivedListener(params, scenario, vehicle, **kwargs):
        Informs listener that a submitted problem has been accepted.
        **kwargs just parrots back any extra arguments you provided when submitting the problem.
    stepPerformedListener(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments, **kwargs):
        Informs listener of the result of the last step
        **kwargs just parrots back any extra arguments you provided when submitting the problem.
    Synchronization:
    Processing is done in a background thread.  When a new problem is submitted, a previous step may be in the process of being reported.
    Manager guarantees that if a new problem is submitted, that it will not call stepPerformedListener(), for the previous problem,
    once it calls inputReceivedListener() for the new problem.  It will call inputReceivedListener() in a timely manner.
    """

    def __init__(self):
        self._lock = Condition(Lock())
        
        self._stepsToPerform = 0
        self._shutdown = False
        self._newInput = None
        self._activePathFinder = None
        self._activeKWArgs = None
        self._thread = Thread(target=self._run)
        self._thread.start()

    def setListeners(self, inputAcceptedListener, stepPerformedListener):
        """
        Must be called before any problem is submitted!
        """
        self._inputAcceptedListener = inputAcceptedListener
        self._stepPerformedListener = stepPerformedListener
        
    def shutdown(self):
        """
        Shutdown the path finding manager and cancel all remaining steps.
        """
        with self._lock:
            self._shutdown = True
            self._lock.notifyAll()

    def shutdownAndWait(self):
        """
        Shutdown and wait for termination.
        """
        self.shutdown()
        self._thread.join()

    def submitProblem(self, params, scenario, vehicle, **kwArgs):
        """
        Submit a new path finding problem.  Will cancel any queued steps.  
        If a step is currently executing, its result will not be published.
        
        Arguments already in local coordinates.  
        The given GPS reference was used for the conversion and will be retained and passed to publish()
        methods to convert back.
        """
        with self._lock:
            # Remember input for publishing purposes
            self._newInput = (params, scenario, vehicle)
            self._activeKWArgs = kwArgs
            self._stepsToPerform = 0
            self._lock.notifyAll()

    def stepProblem(self, numSteps=1):
        """
        Queue up additional steps. Path finder perform steps until:
        1. All queue steps have been executed
        2. A solution is found
        3. A new problem is submitted
        """
        with self._lock:
            if self._activePathFinder is not None or self._newInput is not None:
                self._stepsToPerform += numSteps
                self._lock.notifyAll()

    def _run(self):
        try:
            while True:
                self._processState(*self._getState())
        except self.ShutdownException:
            pass

    def _checkShutdown(self):
        if self._shutdown:
            raise self.ShutdownException            

    def _getState(self):
        """
        Waits until there is a step to perform and returns the path finder to perform it on.
        If a new set of a parameters was submitted a new path finder will be created and input parameters will be published.
        May throw a ShutdownException instead if manager is shutting down
        """
        with self._lock:
            self._checkShutdown()
            
            while self._newInput is None and (self._stepsToPerform == 0 or self._activePathFinder is None):
                self._lock.wait()
                self._checkShutdown()
            
            # extract state to act on before exiting lock
            newInput = self._newInput
            if self._newInput is not None:
                self._activePathFinder = PathFinder(*self._newInput)
                self._newInput = None

            if self._stepsToPerform > 0:
                pathFinderToStep = self._activePathFinder
            else:
                pathFinderToStep = None
            kwargs = self._activeKWArgs
            return (newInput, pathFinderToStep, kwargs)
    
    def _processState(self, newInput, pathFinderToStep, kwargs):
        """
        Runs a step on the given path finder instance.  Afterwards one of 3 things can happen:
        1. Can throw a ShutdownException if manager was shutdown.
        2. Can inform listener of the result of the step
        3. Can find that a new problem was submitted while computing step, in which case the result of the step is not reported (thrown away)
        """
        
        if newInput is not None:
            self._inputAcceptedListener(*newInput, **kwargs)
        
        if pathFinderToStep is not None:
            # Calculate a step for the given path finder.  This takes non-zero time and is therefore not syncrhonized.
            isFinished = not pathFinderToStep.step()
        
            with self._lock:
                # If shutdown, then throw exception and exit
                self._checkShutdown()
                
                # New input already accepted, throw away result of this step
                if pathFinderToStep is not self._activePathFinder or self._newInput is not None:
                    return 
                
                # Report the result of the step to listeners.
                bestPath = pathFinderToStep.getBestPath()
                (previousPathSegments, futurePathSegments, filteredPathSegments) = pathFinderToStep.getDebugData()
                
                if isFinished:
                    self.steps = 0
                    self._activePathFinder = None
                else:
                    self._stepsToPerform -= 1
            
            self._stepPerformedListener(isFinished, bestPath, previousPathSegments, futurePathSegments, filteredPathSegments, **kwargs)

    class ShutdownException(BaseException):
        pass
