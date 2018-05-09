from pathfinding.msg._Vehicle import Vehicle

from demos import DynamicPathFindingVisualizer
from engine import DynamicPathFinder
import engine
from engine.geometry import calcs
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.vehicleInput import VehicleInput
import gui
import numpy as np

startPoint = (95, 95)
endPoint = (5, 5)
maxSpeed = 5.0
startVelocity = calcs.unit(np.array([-1.0, -1.0], np.double)) * maxSpeed
startVelocity = (startVelocity[0], startVelocity[1])
boundaryPoints = [(0, 0), (0, 100), (100, 100), (100, 0)]

noFlyZones = engine.utils.genRandomNoFlyZoneInputsHard(50, 10, 10, 80, 80, 0.01, 0.1, minSpeed=0.0, maxSpeed=2.0,
                                                            startPoint=startPoint, endPoint=endPoint, averageSpeed=maxSpeed)

roads = []
wayPoints = [endPoint]
scenario = ScenarioInput(boundaryPoints, noFlyZones, roads, startPoint, startVelocity, wayPoints)
vehicle = VehicleInput(maxSpeed=5.0, acceleration=1.0)

# Example of loading scenario
# inputDict = engine.loadInput("../scenarios/test1.json")
# scenario = inputDict[SCENARIO_KEY]
# vehicle = inputDict[VEHICLE_KEY]

pathFinder = DynamicPathFinder(scenario, vehicle)

bounds = scenario.calcBounds()
centerX = (bounds[0] + bounds[2]) / 2.0
centerY = (bounds[1] + bounds[3]) / 2.0
rangeX = bounds[2] - bounds[0]
rangeY = bounds[3] - bounds[1]

# Show a slight extra buffer around the border
pathFindingVisualizer = DynamicPathFindingVisualizer(pathFinder, scenario, 800, 800, centerX, centerY, rangeX + 1,
                                                     rangeY + 1)

gui.startGUI()
