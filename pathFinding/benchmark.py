"""
Benchmarks running the path-finder.
"""
from engine import interface
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY
from engine.pathFinder import PathFinder
from utils import profile

input = interface.loadInput("../scenarios/benchmark_veryeasy.json")
pathFinder = PathFinder(input[SCENARIO_KEY], input[VEHICLE_KEY])
pathFinder.findPath()
profile.printAggregate()
profile.reset()
input = interface.loadInput("../scenarios/benchmark_veryhard.json")
pathFinder = PathFinder(input[SCENARIO_KEY], input[VEHICLE_KEY])
for i in range(25):
    pathFinder.step()
profile.printAggregate()
