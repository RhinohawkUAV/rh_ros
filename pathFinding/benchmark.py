"""
Benchmarks running the path-finder.
"""
from engine import interface
from engine.interface.fileUtils import SCENARIO_KEY
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.vehicle import DEFAULT_VEHICLE
from engine.pathFinder import PathFinder
from utils import profile

inputDict = interface.loadInput("../scenarios/benchmark.json")
pathFinder = PathFinder(DEFAULT_PARAMS, inputDict[SCENARIO_KEY], DEFAULT_VEHICLE)
pathFinder.findPath()
profile.printAggregate()
# profile.reset()
# input = interface.loadInput("../scenarios/benchmark_veryhard.json")
# pathFinder = PathFinder(DEFAULT_PARAMS, input[SCENARIO_KEY], DEFAULT_VEHICLE)
# for i in range(25):
#     pathFinder.step()
# profile.printAggregate()
