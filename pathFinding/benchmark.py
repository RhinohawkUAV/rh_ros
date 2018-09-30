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
    bestPath = pathFinder.getBestPath()

    start = time.time()
    while pathFinder.step():
        path = pathFinder.getBestPath()
        if path.isComplete and path.timeThroughHeuristic < bestPath.timeThroughHeuristic:
            bestPath = path
            print "Solution: " + str(bestPath.timeThroughHeuristic) + " -- found at time: " + str(time.time() - start)

    profile.printAggregate()

