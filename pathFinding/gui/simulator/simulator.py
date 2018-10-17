import copy
import math
from threading import Thread, Condition, Lock
import time

from engine.geometry import calcs
from gui import draw
import gui
from gui.core import Drawable
import numpy as np


class SimManager:

    def __init__(self):
        self._lock = Condition(Lock())
        self._simulator = None
        self._shutdown = False
        self._speed = 1.0
        self._thread = Thread(target=self._run)
        self._thread.start()

    def setState(self, scenario, vehicle):
        with self._lock:
            self._simulator = Simulator(scenario, vehicle)

    def scaleSpeed(self, scale):
        with self._lock:
            self._speed *= scale
        
    def setPath(self, path):
        with self._lock:
            if self._simulator is not None:
                if self._simulator.setPath(path):
                    self._speed = 1.0

    def copyState(self):
        with self._lock:
            return copy.deepcopy(self._simulator)
    
    def addCircularNFZs(self, nfzs):
        with self._lock:
            self._simulator.addCircularNFZs(nfzs)
        
    def addPolyNFZs(self, nfzs):
        with self._lock:
            self._simulator.addPolyNFZs(nfzs)            

    def _step(self, now):
        elapsedTime = now - self._lastStepTime
        self._lastStepTime = now
        with self._lock:
            if self._simulator is not None:
                self._simulator.step(elapsedTime * self._speed)
      
    def _run(self):
        try:
            i = 0
            self._lastStepTime = time.time()
            while True:
                self._checkShutdown()
                now = time.time()
                self._step(now)
                i += 1
#                 print i
                
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


COURSE_WAYPOINT_RADIUS = 100.0


class Simulator(Drawable):

    def __init__(self, scenario, vehicle):
        self.scenario = scenario
        self._vehicle = vehicle
        
        # Waypoints given in the path finder solution (not competition waypoints)
        self._pathWaypointIndex = 0
        self._path = None
    
    def setPath(self, path):
        changed = True
        if self._path is not None and len(path.pathWaypoints) > 0:
            (goal, radius) = self._currentPathWaypoint()     
            if goal is not None and calcs.length(goal - path.pathWaypoints[0].position) < 1.0:
                changed = False
        self._path = path
        self._pathWaypointIndex = 0
        return changed
    
    def addPolyNFZs(self, nfzs):
        self._polyNFZs.extend(nfzs)
        
    def addCircularNFZs(self, nfzs):
        self._circularNFZs.extend(nfzs)
        
    def step(self, simTime):
        self._updateNFZs(simTime)
        position = self.scenario.startPoint
        velocity = self.scenario.startVelocity
        self._advanceGoal(position)
        pathPoint = self._advancePath(position)
        if pathPoint is None:
            acceleration = np.array((0, 0), np.double)
        else:
            acceleration = self._steer(pathPoint, position, velocity, simTime)
            
        position += 0.5 * acceleration * simTime * simTime + velocity * simTime
        velocity += acceleration * simTime
    
        self.scenario.startPoint = position
        self.scenario.startVelocity = velocity

    def _currentPathWaypoint(self):
        if self._pathWaypointIndex == len(self._path.pathWaypoints):
            return (None, None)
        else:
            return (self._path.pathWaypoints[self._pathWaypointIndex].position, self._path.pathWaypoints[self._pathWaypointIndex].radius)

    def _pathPointReached(self, position, goal, radius):
        return goal is not None and calcs.length(position - goal) <= radius
            
    def _advancePath(self, position):
        if self._path is None:
            return None

        (goal, radius) = self._currentPathWaypoint()            
        while self._pathPointReached(position, goal, radius):
            self._pathWaypointIndex += 1
            (goal, radius) = self._currentPathWaypoint()
        return goal
    
    def _advanceGoal(self, position):
        if self.scenario is None:
            return
        
        while len(self.scenario.wayPoints) > 0 and calcs.length(self.scenario.wayPoints[0] - position) <= COURSE_WAYPOINT_RADIUS:
            self.scenario.wayPoints = self.scenario.wayPoints[1:]
        
    def _calcSpeedDelta(self, turnAngle, speed, desiredSpeed):
        return math.sqrt(speed * speed + desiredSpeed * desiredSpeed - 2 * speed * desiredSpeed * math.cos(turnAngle))
        
    def _steer(self, goal, position, velocity, simTime):
        
        (direction, speed) = calcs.unitAndLength(velocity)
        turnAngle = calcs.modAngleSigned(calcs.relativeAngle(velocity, goal - position))
        
        speedDelta = self._vehicle.maxSpeed - speed
        
        maxTurn = self._vehicle.acceleration * simTime / speed
        
        turnAngle = calcs.clampMag(turnAngle, maxTurn)
        
        turnAcceleration = turnAngle * speed / simTime
        availableAcceleration = self._vehicle.acceleration * self._vehicle.acceleration - turnAcceleration * turnAcceleration
        if availableAcceleration < 0.0:
            availableAcceleration = 0.0
        availableAcceleration = math.sqrt(availableAcceleration)

        speedAcceleration = calcs.clampMag(speedDelta / simTime, availableAcceleration)
        
        return turnAcceleration * calcs.CCWNorm(direction) + speedAcceleration * direction
        
#         toGoal = calcs.unit(goal - position)
#         velDiff = toGoal * calcs.length(velocity) - velocity
#         (accDir, velDiffMag) = calcs.unitAndLength(velDiff)
#         
#         if velDiffMag / simTime > self._vehicle.acceleration:
#             return accDir * self._vehicle.acceleration
#         else:
#             return velDiff / simTime
        
    def _updateNFZs(self, simTime):
        
        for i in range(len(self.scenario.noFlyZones)):
            self.scenario.noFlyZones[i] = self.scenario.noFlyZones[i].copyAtTime(simTime)
            
        for i in range(len(self.scenario.dynamicNoFlyZones)):
            self.scenario.dynamicNoFlyZones[i] = self.scenario.dynamicNoFlyZones[i].copyAtTime(simTime)
            
    def draw(self, visualizer, solutionColor="green", solutionWidth=1.0, **kwargs):
        if self._path is not None:
            for pathSegment in self._path.pathSegments:
                pathSegment.draw(visualizer, color=solutionColor, width=solutionWidth)            
        
            for solutionWaypoint in self._path.pathWaypoints:
                gui.draw.drawCircle(visualizer, solutionWaypoint.position, solutionWaypoint.radius, color=solutionColor)
        draw.drawNoFlyZones(visualizer, self.scenario.noFlyZones, color="red", width=1.0, **kwargs)
        draw.drawDynamicNoFlyZones(visualizer, self.scenario.dynamicNoFlyZones, color="red", **kwargs)
        draw.drawPoly(visualizer, self.scenario.boundaryPoints, color="red")

        draw.drawVelocity(visualizer, self.scenario.startPoint, self.scenario.startVelocity, color="orange")
        draw.drawPoint(visualizer, self.scenario.startPoint, radius=30.0, color="orange")
