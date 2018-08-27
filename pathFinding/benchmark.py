"""
Benchmarks running the path-finder.
"""
import time

from engine import interface
from engine.interface.fileUtils import SCENARIO_KEY
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.vehicle import DEFAULT_VEHICLE
from engine.pathFinder import PathFinder
from utils import profile

if __name__ == "__main__":
    inputDict = interface.loadInput("../scenarios/veryhard.json")
    pathFinder = PathFinder(DEFAULT_PARAMS, inputDict[SCENARIO_KEY], DEFAULT_VEHICLE)
    start = time.time()
    while not pathFinder.isDone():
        pathFinder.step()
        if pathFinder.solutionUpdated():
            (wayPoints, pathSegments) = pathFinder.getSolution()
            print "Solution: " + str(pathSegments[-1].startTime + pathSegments[-1].elapsedTime) + " -- found at time: " + str(time.time() - start)
    
    profile.printAggregate()
# profile.reset()
# input = interface.loadInput("../scenarios/benchmark_veryhard.json")
# pathFinder = PathFinder(DEFAULT_PARAMS, input[SCENARIO_KEY], DEFAULT_VEHICLE)
# for i in range(25):
#     pathFinder.step()
# profile.printAggregate()
