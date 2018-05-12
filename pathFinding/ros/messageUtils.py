"""
Utilities for converting ROS messages to/from inputs to the path-finder.
"""
from pathfinding.msg._NoFlyZone import NoFlyZone
from pathfinding.msg._Road import Road
from pathfinding.msg._Scenario import Scenario
from pathfinding.msg._Vec2 import Vec2
from pathfinding.msg._Vehicle import Vehicle

from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY, loadInput
from engine.interface.noFlyZoneInput import NoFlyZoneInput
from engine.interface.roadInput import RoadInput
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.vehicleInput import VehicleInput

#**********************Msg objects to input objects********************


def msgToVehicle(msg):
    return VehicleInput(float(msg.maxSpeed), float(msg.acceleration))


def msgToScenario(msg):
    noFlyZones = []
    for noFlyZone in msg.noFlyZones:
        noFlyZones.append(msgToNoFlyZone(noFlyZone)) 
    roads = []
    for road in msg.roads:
        roads.append(msgToRoad(road))
    
    return ScenarioInput(msgToPointList(msg.boundaryPoints),
                    noFlyZones,
                    roads,
                    msgToPoint(msg.startPoint),
                    msgToPoint(msg.startVelocity),
                    msgToPointList(msg.wayPoints))


def msgToRoad(msg):
    return RoadInput(msgToPoint(msg.startPoint), msgToPoint(msg.startPoint), float(msg.width))


def msgToNoFlyZone(msg):
    return NoFlyZoneInput(msgToPointList(msg.points), msgToPoint(msg.velocity))

    
def msgToPointList(msg):
    pointList = []
    for pointMsg in msg:
        pointList.append(msgToPoint(pointMsg))
    return pointList


def msgToPoint(msg):
    return (float(msg.x), float(msg.y))


#**********************Input objects to msg objects********************
def vehicleToMsg(vehicle):
    msg = Vehicle()
    msg.maxSpeed = vehicle.maxSpeed
    msg.acceleration = vehicle.acceleration


def scenarioToMsg(scenario):
    msg = Scenario()
    msg.boundaryPoints = pointListToMsg(scenario.boundaryPoints)
    msg.noFlyZones = []
    for noFlyZone in scenario.noFlyZones:
        msg.noFlyZones.append(nfzToMsg(noFlyZone))
    for road in scenario.roads:
        msg.roads.append(roadToMsg(road))
    msg.startPoint = pointToMsg(scenario.startPoint)
    msg.startVelocity = pointToMsg(scenario.startVelocity)
    msg.wayPoints = pointListToMsg(scenario.wayPoints)
    return msg
        

def roadToMsg(road):
    msg = Road()
    msg.startPoint = pointToMsg(road.startPoint)
    msg.endPoint = pointToMsg(road.endPoint)
    msg.width = road.width
    return msg

    
def nfzToMsg(noFlyZone):
    msg = NoFlyZone()
    msg.points = pointListToMsg(noFlyZone.points)
    msg.velocity = pointToMsg(noFlyZone.velocity)
    return msg


def pointListToMsg(points):
    msg = []
    for point in points:
        msg.append(pointToMsg(point))
    return msg

        
def pointToMsg(point):
    return Vec2(point[0], point[1])


def msgToInput(scenarioMsg, vehicleMsg):
    inputDict = {}
#     boundaryPoints = vecListToPointList(scenarioMsg.boundaryPoints)
    scenario = ScenarioInput(scenarioMsg.boundaryPoints,
                             scenarioMsg.noFlyZones,
                             scenarioMsg.roads,
                             scenarioMsg.startPoint,
                             scenarioMsg.startVelocity,
                             scenarioMsg.wayPoints
                             )
    vehicle = None
    inputDict[SCENARIO_KEY] = scenario
    
    inputDict[VEHICLE_KEY] = vehicle
