"""
Benchmarks running the path-finder.
"""
import time

from engine import interface
from engine.pathFinder import PathFinder
from utils import profile


def benchmarkScenario(fileName):
    
    (params, scenario, vehicle) = interface.load(fileName)
    pathFinder = PathFinder(params, scenario, vehicle)
    bestPath = pathFinder.getBestPath()
 
    print "Benchmarking: " + fileName
    start = time.time()
    while pathFinder.step():
        path = pathFinder.getBestPath()
        if path.quality >= 2 and path.estimatedTime < bestPath.estimatedTime:
            bestPath = path
            print "Solution: " + str(bestPath.estimatedTime) + " -- found at time: " + str(time.time() - start)
 
    profile.printAggregate()


if __name__ == "__main__":
    benchmarkScenario("../scenarios/unnecessary_loop_simple.json")
    benchmarkScenario("../scenarios/dynamic1.json")
    benchmarkScenario("../scenarios/dynamic2.json")
    benchmarkScenario("../scenarios/occluded.json")

