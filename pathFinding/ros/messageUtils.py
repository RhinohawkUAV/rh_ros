"""
Utilities for converting ROS messages to/from inputs to the path-finder.
"""
from pathfinding.msg._Arc import Arc
from pathfinding.msg._NoFlyZone import NoFlyZone
from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSegment import PathSegment
from pathfinding.msg._Road import Road
from pathfinding.msg._Scenario import Scenario
from pathfinding.msg._Vec2 import Vec2
from pathfinding.msg._Vehicle import Vehicle

import engine
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY, loadInput
from engine.interface.noFlyZoneInput import NoFlyZoneInput
from engine.interface.roadInput import RoadInput
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.vehicleInput import VehicleInput


#**********************Msg objects to path finding objects********************
def msgToInput(scenarioMsg, vehicleMsg):
    inputDict = {}
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


def msgToPathDebug(msg):
    pastPathSegments = msgToPathSegmentList(msg.pastPathSegments)
    futurePathSegments = msgToPathSegmentList(msg.futurePathSegments)
    filteredPathSegments = msgToPathSegmentList(msg.filteredPathSegments)
    return (pastPathSegments, futurePathSegments, filteredPathSegments)


def msgToPathSegmentList(msg):
    pathSegments = []
    for pathSegmentMsg in msg:
        pathSegments.append(msgToPathSegment(pathSegmentMsg))
    return pathSegments


def msgToPathSegment(msg):
    return ArcPathSegment(float(msg.startTime),
                                 float(msg.elapsedTime),
                                 msgToPoint(msg.endPoint),
                                 float(msg.speed),
                                 msgToArc(msg.arc))

            
def msgToArc(msg):
    return engine.geometry.arc.Arc(float(msg.direction),
                                  float(msg.radius),
                                  msgToPoint(msg.center),
                                  float(msg.start),
                                  float(msg.length))


def msgToPoint(msg):
    return (float(msg.x), float(msg.y))


#**********************Path finding objects to msg objects********************
def vehicleToMsg(vehicle):
    msg = Vehicle()
    msg.maxSpeed = vehicle.maxSpeed
    msg.acceleration = vehicle.acceleration
    return msg


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


def pathDebugToMsg(pastPathSegments, futurePathSegments, filteredPathSegments):
    msg = PathDebug()
    msg.pastPathSegments = pathSegmentListToMsg(pastPathSegments)
    msg.futurePathSegments = pathSegmentListToMsg(futurePathSegments)
    msg.filteredPathSegments = pathSegmentListToMsg(filteredPathSegments)
    return msg


def pathSegmentListToMsg(pathSegments):
    msg = []
    for pathSegment in pathSegments:
        msg.append(pathSegmentToMsg(pathSegment))
    return msg


def pathSegmentToMsg(pathSegment):
    if isinstance(pathSegment, ArcPathSegment):
        msg = PathSegment()
        msg.startTime = pathSegment.startTime
        msg.elapsedTime = pathSegment.elapsedTime
        msg.speed = pathSegment.speed
        msg.arc = arcToMsg(pathSegment.arc)
        msg.endPoint = pointToMsg(pathSegment.endPoint)
        msg.endSpeed = pathSegment.endSpeed
        return msg
    else:
        raise "Path segment type not suppored by messages"

            
def arcToMsg(arc):
    msg = Arc()
    msg.direction = arc.rotDirection
    msg.radius = arc.radius
    msg.center = pointToMsg(arc.center)
    msg.start = arc.start
    msg.length = arc.length
    return msg

    
def pointToMsg(point):
    return Vec2(point[0], point[1])

