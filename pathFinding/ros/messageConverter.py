"""
Utilities for converting ROS messages to/from inputs to the path-finder.
"""
from pathfinding.msg._Arc import Arc
from pathfinding.msg._NoFlyZone import NoFlyZone
from pathfinding.msg._PathDebug import PathDebug
from pathfinding.msg._PathSegment import PathSegment
from pathfinding.msg._Road import Road
from pathfinding.msg._Scenario import Scenario
from pathfinding.msg._Vehicle import Vehicle

import engine
from engine.geometry.pathSegment.arcPathSegment import ArcPathSegment
from engine.interface.fileUtils import SCENARIO_KEY, VEHICLE_KEY
from engine.interface.gpsTransform.gpsTransform import GPSTransformer
from engine.interface.noFlyZoneInput import NoFlyZoneInput
from engine.interface.roadInput import RoadInput
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.vehicleInput import VehicleInput
import numpy as np


#**********************Msg objects to path finding objects********************
class MessageConverter:

    def __init__(self, gpsRef):
        self._gpsTransformer = GPSTransformer(gpsRef)
        
    def msgToInput(self, scenarioMsg, vehicleMsg):
        inputDict = {}
        scenario = ScenarioInput(scenarioMsg.boundaryPoints,
                                 scenarioMsg.noFlyZones,
                                 scenarioMsg.roads,
                                 scenarioMsg.startPoint,
                                 scenarioMsg.startVelocity,
                                 scenarioMsg.wayPoints
                                 )
        vehicle = self.msgToVehicle(vehicleMsg)
        inputDict[SCENARIO_KEY] = scenario
        inputDict[VEHICLE_KEY] = vehicle
    
    def msgToVehicle(self, msg):
        return VehicleInput(float(msg.maxSpeed), float(msg.acceleration))
    
    def msgToScenario(self, msg):
        noFlyZones = []
        for noFlyZone in msg.noFlyZones:
            noFlyZones.append(self.msgToNoFlyZone(noFlyZone)) 
        roads = []
        for road in msg.roads:
            roads.append(self.msgToRoad(road))
        
        return ScenarioInput(self.msgToPointList(msg.boundaryPoints),
                        noFlyZones,
                        roads,
                        self.msgToPoint(msg.startPoint),
                        self.msgToVector(msg.startVelocity),
                        self.msgToPointList(msg.wayPoints))
    
    def msgToRoad(self, msg):
        return RoadInput(self.msgToPoint(msg.startPoint), self.msgToPoint(msg.startPoint), float(msg.width))
    
    def msgToNoFlyZone(self, msg):
        return NoFlyZoneInput(self.msgToPointList(msg.points), self.msgToPoint(msg.velocity))
        
    def msgToPointList(self, msg):
        pointList = []
        for pointMsg in msg:
            pointList.append(self.msgToPoint(pointMsg))
        return pointList
    
    def msgToPathDebug(self, msg):
        pastPathSegments = self.msgToPathSegmentList(msg.pastPathSegments)
        futurePathSegments = self.msgToPathSegmentList(msg.futurePathSegments)
        filteredPathSegments = self.msgToPathSegmentList(msg.filteredPathSegments)
        return (pastPathSegments, futurePathSegments, filteredPathSegments)
    
    def msgToPathSegmentList(self, msg):
        pathSegments = []
        for pathSegmentMsg in msg:
            pathSegments.append(self.msgToPathSegment(pathSegmentMsg))
        return pathSegments
    
    def msgToPathSegment(self, msg): 
#         (self, startTime, elapsedTime, lineStartPoint, endPoint, endSpeed, endDirection, arc):
        endVelocity = self.msgToVector(msg.endVelocity)
        endSpeed = np.linalg.norm(endVelocity)
        endUnitVelocity = endVelocity / endSpeed
        return ArcPathSegment(float(msg.startTime),
                                     float(msg.elapsedTime),
                                     self.msgToPoint(msg.lineStartPoint),
                                     self.msgToPoint(msg.endPoint),
                                     endSpeed,
                                     endUnitVelocity,
                                     self.msgToArc(msg.arc))
                
    def msgToArc(self, msg):
        start = self._gpsTransformer.gpsAngleToLocal(float(msg.start))

        return engine.geometry.arc.Arc(-float(msg.direction),
                                      float(msg.radius),
                                      self.msgToPoint(msg.center),
                                      start,
                                      float(msg.length))
    
    #**********************Path finding objects to msg objects********************
    def vehicleToMsg(self, vehicle):
        msg = Vehicle()
        msg.maxSpeed = vehicle.maxSpeed
        msg.acceleration = vehicle.acceleration
        return msg
    
    def scenarioToMsg(self, scenario):
        msg = Scenario()
        msg.boundaryPoints = self.pointListToMsg(scenario.boundaryPoints)
        msg.noFlyZones = []
        for noFlyZone in scenario.noFlyZones:
            msg.noFlyZones.append(self.nfzToMsg(noFlyZone))
        for road in scenario.roads:
            msg.roads.append(self.roadToMsg(road))
        msg.startPoint = self.pointToMsg(scenario.startPoint)
        msg.startVelocity = self.vectorToMsg(scenario.startVelocity)
        msg.wayPoints = self.pointListToMsg(scenario.wayPoints)
        return msg
    
    def roadToMsg(self, road):
        msg = Road()
        msg.startPoint = self.pointToMsg(road.startPoint)
        msg.endPoint = self.pointToMsg(road.endPoint)
        msg.width = road.width
        return msg
        
    def nfzToMsg(self, noFlyZone):
        msg = NoFlyZone()
        msg.points = self.pointListToMsg(noFlyZone.points)
        msg.velocity = self.pointToMsg(noFlyZone.velocity)
        return msg
    
    def pointListToMsg(self, points):
        msg = []
        for point in points:
            msg.append(self.pointToMsg(point))
        return msg
    
    def pathDebugToMsg(self, pastPathSegments, futurePathSegments, filteredPathSegments):
        msg = PathDebug()
        msg.pastPathSegments = self.pathSegmentListToMsg(pastPathSegments)
        msg.futurePathSegments = self.pathSegmentListToMsg(futurePathSegments)
        msg.filteredPathSegments = self.pathSegmentListToMsg(filteredPathSegments)
        return msg
    
    def pathSegmentListToMsg(self, pathSegments):
        msg = []
        for pathSegment in pathSegments:
            msg.append(self.pathSegmentToMsg(pathSegment))
        return msg
    
    def pathSegmentToMsg(self, pathSegment):
        if isinstance(pathSegment, ArcPathSegment):
            msg = PathSegment()
            msg.startTime = pathSegment.startTime
            msg.elapsedTime = pathSegment.elapsedTime
            msg.speed = pathSegment.speed
            msg.arc = self.arcToMsg(pathSegment.arc)
            msg.lineStartPoint = self.pointToMsg(pathSegment.lineStartPoint)
            msg.endPoint = self.pointToMsg(pathSegment.endPoint)
            msg.endVelocity = self.vectorToMsg(pathSegment.endUnitVelocity * pathSegment.endSpeed)
            return msg
        else:
            raise "Path segment type not suppored by messages"
                
    def arcToMsg(self, arc):
        msg = Arc()
        msg.direction = -arc.rotDirection
        msg.radius = arc.radius
        msg.center = self.pointToMsg(arc.center)
        msg.start = self._gpsTransformer.localAngleToGPS(arc.start)
        msg.length = arc.length
        return msg

#***************************Point/vector Conversions*****************************

    def msgToPoint(self, msg):
        return self._gpsTransformer.gpsToLocal(msg)
         
    def pointToMsg(self, point):
        return self._gpsTransformer.localToGPS(point)
 
    def msgToVector(self, msg):
        return self._gpsTransformer.gpsVelocityToLocal(msg)
         
    def vectorToMsg(self, vec):
        return self._gpsTransformer.localToGPSVelocity(vec)
     
