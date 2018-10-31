"""
Benchmarks running the path-finder.
"""
import time

from constants import COURSE_DIM
from engine import interface
from engine.geometry import calcs
from engine.interface.generator import ObstacleGenerator
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.scenario import Scenario
from engine.interface.vehicle import DEFAULT_VEHICLE
from engine.pathFinder import PathFinder
import numpy as np
from utils import profile


def solve(params, scenario, vehicle, timeout=20.0):
    pathFinder = PathFinder(params, scenario, vehicle)
    
    start = time.time()
    while pathFinder.step() and time.time() - start < timeout:
        pass
    return pathFinder.getBestPath()

    
if __name__ == "__main__":
    params = DEFAULT_PARAMS
    vehicle = DEFAULT_VEHICLE
    boundaryPoints = np.array([(-COURSE_DIM / 2.0, -COURSE_DIM / 2.0),
                          (-COURSE_DIM / 2.0, COURSE_DIM / 2.0),
                          (COURSE_DIM / 2.0, COURSE_DIM / 2.0),
                          (COURSE_DIM / 2.0, -COURSE_DIM / 2.0)], np.double)
    startPoint = boundaryPoints[0] * 0.8
    waypoints = []
    waypoints.append(boundaryPoints[2] * 0.8)
    waypoints.append(boundaryPoints[1] * 0.8)
    waypoints.append(boundaryPoints[3] * 0.8)
    startVelocity = calcs.unit(waypoints[0] - startPoint) * DEFAULT_VEHICLE.maxSpeed
    scenario = Scenario(boundaryPoints=boundaryPoints,
                         noFlyZones=[],
                         dynamicNoFlyZones=[],
                         roads=[],
                         startPoint=startPoint,
                         startVelocity=startVelocity,
                         wayPoints=waypoints)
 
    for i in range(100):
        bestPath = solve(params, scenario, vehicle)
        
        generator = ObstacleGenerator(params, scenario, vehicle)
        generator.setGenerationInfo(COURSE_DIM / 10.0,
                        1.0,
                        0.0)
        generator.block(bestPath.pathSegments)
        interface.save("../scenarios/generated.json", params, scenario, vehicle)
        print "Solution (" + str(i) + "): " + str(bestPath.estimatedTime)
 
    profile.printAggregate()

