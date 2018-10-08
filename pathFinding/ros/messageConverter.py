"""
Utilities for converting ROS messages to/from inputs to the path-finder.
"""
import math

from engine.geometry.obstacle.arcFinder.arcCalc import ArcCalc
from engine.geometry.obstacle.arcFinder.arcPathSegment import ArcPathSegment
import engine.interface.dynamicNoFlyZone
from engine.interface.gpsTransform.gpsTransform import GPSTransformer
import engine.interface.noFlyZone
from engine.interface.outputPath import OutputPath
import engine.interface.pathFindParams
import engine.interface.road
import engine.interface.scenario
import engine.interface.solutionWaypoint
import engine.interface.vehicle
import numpy as np
import pathfinding.msg as pfm


#**********************Msg objects to path finding objects********************
class MessageConverter:

    def __init__(self, gpsRef):
        self._gpsTransformer = GPSTransformer(gpsRef)

    def msgToInput(self, inputMsg):
        params = self.msgToParams(inputMsg.params)
        scenario = self.msgToScenario(inputMsg.scenario)
        vehicle = self.msgToVehicle(inputMsg.vehicle)
        return (params, scenario, vehicle)
        
    def msgToParams(self, inputParamsMsg):
        return engine.interface.pathFindParams.PathFindParams(\
                float(inputParamsMsg.waypointAcceptanceRadii), \
                float(inputParamsMsg.nfzBufferWidth),
                float(inputParamsMsg.nfzTargetOffset),
                float(inputParamsMsg.vertexHeuristicMultiplier))
    
    def msgToVehicle(self, msg):
        return engine.interface.vehicle.Vehicle(float(msg.maxSpeed), float(msg.acceleration))
    
    def msgToScenario(self, msg):
        noFlyZones = []
        for noFlyZone in msg.noFlyZones:
            noFlyZones.append(self.msgToNoFlyZone(noFlyZone)) 
        dynamicNoFlyZones = []
        for dNoFlyZone in msg.dynamicNoFlyZones:
            dynamicNoFlyZones.append(self.msgToDynamicNoFlyZone(dNoFlyZone)) 
            
        roads = []
        for road in msg.roads:
            roads.append(self.msgToRoad(road))
        
        return engine.interface.scenario.Scenario(
                        self.msgToPointList(msg.boundaryPoints),
                        noFlyZones,
                        dynamicNoFlyZones,
                        roads,
                        self.msgToPoint(msg.startPoint),
                        self.msgToVector(msg.startVelocity),
                        self.msgToPointList(msg.wayPoints))
    
    def msgToRoad(self, msg):
        return engine.interface.road.Road(self.msgToPoint(msg.startPoint), self.msgToPoint(msg.startPoint), float(msg.width))
    
    def msgToNoFlyZone(self, msg):
        return engine.interface.noFlyZone.NoFlyZone(
            self.msgToPointList(msg.points),
            self.msgToVector(msg.velocity),
            int(msg.ID))

    def msgToDynamicNoFlyZone(self, msg):
        return engine.interface.dynamicNoFlyZone.DynamicNoFlyZone(
            self.msgToPoint(msg.center),
            float(msg.radius),
            self.msgToVector(msg.velocity),
            int(msg.ID))
        
    def msgToPointList(self, msg):
        pointList = []
        for pointMsg in msg:
            pointList.append(self.msgToPoint(pointMsg))
        return pointList
    
    def msgToPathDebug(self, msg):
        isFinished = msg.isFinished
        bestPath = self.msgToOutputPath(msg.bestPath)
        pastPathSegments = self.msgToPathSegmentList(msg.pastPathSegments)
        futurePathSegments = self.msgToPathSegmentList(msg.futurePathSegments)
        filteredPathSegments = self.msgToPathSegmentList(msg.filteredPathSegments)
        return (isFinished, bestPath, pastPathSegments, futurePathSegments, filteredPathSegments)
    
    def msgToOutputPath(self, msg):
        solutionWaypoints = self.msgToSolutionWaypointList(msg.solutionWaypoints)
        pathSegments = self.msgToPathSegmentList(msg.solutionPathSegments)
        return OutputPath(solutionWaypoints,
                          pathSegments,
                          int(msg.quality),
                          int(msg.numWaypointsCompleted),
                          float(msg.estimatedTime))

    def msgToSolutionWaypointList(self, msg):
        solutionWaypoints = []
        for solutionWaypointMsg in msg:
            solutionWaypoints.append(self.msgToSolutionWaypoint(solutionWaypointMsg))
        return solutionWaypoints

    def msgToSolutionWaypoint(self, msg):
        return engine.interface.solutionWaypoint.SolutionWaypoint(self.msgToPoint(msg.position), float(msg.radius))
    
    def msgToPathSegmentList(self, msg):
        pathSegments = []
        for pathSegmentMsg in msg:
            pathSegments.append(self.msgToPathSegment(pathSegmentMsg))
        return pathSegments
    
    def msgToPathSegment(self, msg): 
        arc = self.msgToArc(msg.arc, float(msg.speed))
        lineTime = float(msg.elapsedTime) - arc.arcTime()
        return ArcPathSegment(float(msg.startTime), arc, lineTime, 0.0)
                
    def msgToArc(self, msg, speed):
        if msg.length < 0.0:
            rotDirection = 1.0
            length = -math.radians(float(msg.length))
        else:
            rotDirection = -1.0
            length = math.radians(float(msg.length))
            
        start = self._gpsTransformer.gpsAngleToLocal(float(msg.start), rotDirection)

        radius = float(msg.radius)
        return ArcCalc(rotDirection,
                       radius,
                       self.msgToPoint(msg.center),
                       start,
                       length,
                       speed,
                       speed / radius)
    
    #**********************Path finding objects to msg objects********************
    def inputToMsg(self, params, scenario, vehicle):
        msg = pfm.PathInput()
        msg.params = self.paramsToMsg(params)
        msg.scenario = self.scenarioToMsg(scenario)
        msg.vehicle = self.vehicleToMsg(vehicle)
        return msg

    def paramsToMsg(self, inputParams):
        msg = pfm.Params()
        msg.waypointAcceptanceRadii = inputParams.waypointAcceptanceRadii
        msg.nfzBufferWidth = inputParams.nfzBufferWidth
        msg.nfzTargetOffset = inputParams.nfzTargetOffset
        msg.vertexHeuristicMultiplier = inputParams.vertexHeuristicMultiplier
        return msg
    
    def vehicleToMsg(self, vehicle):
        msg = pfm.Vehicle()
        msg.maxSpeed = vehicle.maxSpeed
        msg.acceleration = vehicle.acceleration
        return msg
    
    def scenarioToMsg(self, scenario):
        msg = pfm.Scenario()
        msg.boundaryPoints = self.pointListToMsg(scenario.boundaryPoints)
        msg.noFlyZones = []
        msg.dynamicNoFlyZones = []
        
        for noFlyZone in scenario.noFlyZones:
            msg.noFlyZones.append(self.nfzToMsg(noFlyZone))
        for dNoFlyZone in scenario.dynamicNoFlyZones:
            msg.dynamicNoFlyZones.append(self.dnfzToMsg(dNoFlyZone))
            
        for road in scenario.roads:
            msg.roads.append(self.roadToMsg(road))
        msg.startPoint = self.pointToMsg(scenario.startPoint)
        msg.startVelocity = self.vectorToMsg(scenario.startVelocity)
        msg.wayPoints = self.pointListToMsg(scenario.wayPoints)
        return msg
    
    def roadToMsg(self, road):
        msg = pfm.Road()
        msg.startPoint = self.pointToMsg(road.startPoint)
        msg.endPoint = self.pointToMsg(road.endPoint)
        msg.width = road.width
        return msg
        
    def nfzToMsg(self, noFlyZone):
        msg = pfm.NoFlyZone()
        msg.points = self.pointListToMsg(noFlyZone.points)
        msg.velocity = self.vectorToMsg(noFlyZone.velocity)
        msg.ID = noFlyZone.ID
        return msg
    
    def dnfzToMsg(self, dNoFlyZone):
        msg = pfm.DynamicNoFlyZone()
        msg.center = self.pointToMsg(dNoFlyZone.center)
        msg.radius = dNoFlyZone.radius
        msg.velocity = self.vectorToMsg(dNoFlyZone.velocity)
        msg.ID = dNoFlyZone.ID
        return msg
        
    def pointListToMsg(self, points):
        msg = []
        for point in points:
            msg.append(self.pointToMsg(point))
        return msg
    
    def pathDebugToMsg(self, isFinished, bestPath, pastPathSegments, futurePathSegments, filteredPathSegments):
        msg = pfm.PathDebug()
        msg.isFinished = isFinished
        msg.bestPath = self.outputPathToMsg(bestPath)
        msg.pastPathSegments = self.pathSegmentListToMsg(pastPathSegments)
        msg.futurePathSegments = self.pathSegmentListToMsg(futurePathSegments)
        msg.filteredPathSegments = self.pathSegmentListToMsg(filteredPathSegments)
        return msg

    def outputPathToMsg(self, outputPath):
        msg = pfm.PathSolution()
        msg.solutionWaypoints = self.solutionWaypointListToMsg(outputPath.pathWaypoints)
        msg.solutionPathSegments = self.pathSegmentListToMsg(outputPath.pathSegments)
        msg.quality = outputPath.quality
        msg.numWaypointsCompleted = outputPath.numWayPointsCompleted
        msg.estimatedTime = outputPath.estimatedTime
        return msg

    def solutionWaypointListToMsg(self, solutionWaypoints):
        msg = []
        for solutionWaypoint in solutionWaypoints:
            msg.append(self.solutionWaypointToMsg(solutionWaypoint))
        return msg

    def solutionWaypointToMsg(self, solutionWaypoint):
        msg = pfm.SolutionWaypoint() 
        msg.position = self.pointToMsg(solutionWaypoint.position)
        msg.radius = solutionWaypoint.radius
        return msg
        
    def pathSegmentListToMsg(self, pathSegments):
        msg = []
        for pathSegment in pathSegments:
            msg.append(self.pathSegmentToMsg(pathSegment))
        return msg
    
    def pathSegmentToMsg(self, pathSegment):
        if isinstance(pathSegment, ArcPathSegment):
            msg = pfm.PathSegment()
            msg.startTime = pathSegment.startTime
            msg.elapsedTime = pathSegment.elapsedTime
            msg.speed = pathSegment.endSpeed
            msg.arc = self.arcToMsg(pathSegment.arc)
            msg.lineStartPoint = self.pointToMsg(pathSegment.lineStartPoint)
            msg.endPoint = self.pointToMsg(pathSegment.endPoint)
            msg.endVelocity = self.vectorToMsg(pathSegment.endUnitVelocity * pathSegment.endSpeed)
            return msg
        else:
            raise "Path segment type not suppored by messages"
                
    def arcToMsg(self, arc):
        msg = pfm.Arc()
        msg.radius = arc.radius
        msg.center = self.pointToMsg(arc.center)
        msg.start = self._gpsTransformer.localAngleToGPS(arc.start, arc.rotDirection)
        msg.length = math.degrees(arc.length) * -arc.rotDirection
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
