"""
Benchmarks running the path-finder.
"""
import time

from engine import interface
from engine.pathFinder import PathFinder
from utils import profile

if __name__ == "__main__":

    (params, scenario, vehicle) = interface.load("../scenarios/veryhard.json")
    pathFinder = PathFinder(params, scenario, vehicle)
    bestPath = pathFinder.getBestPath()
 
    start = time.time()
    while pathFinder.step():
        path = pathFinder.getBestPath()
        if path.quality >= 2 and path.estimatedTime < bestPath.estimatedTime:
            bestPath = path
            print "Solution: " + str(bestPath.estimatedTime) + " -- found at time: " + str(time.time() - start)
 
    profile.printAggregate()

