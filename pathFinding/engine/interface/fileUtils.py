"""
Utilities related to loading/saving/generating scenarios.
"""
import json

from engine.interface import pathFindParams
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
from engine.interface.noFlyZone import NoFlyZone
from engine.interface.pathFindParams import PathFindParams
from engine.interface.road import Road
from engine.interface.scenario import Scenario
from engine.interface.vehicle import Vehicle
import numpy as np

INPUT_PARAMS_KEY = "params"
SCENARIO_KEY = "scenario"
VEHICLE_KEY = "vehicle"


class Encoder(json.JSONEncoder):

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif callable(getattr(obj, "toJSONDict", None)):
            return obj.toJSONDict()
        else:
            return obj.__dict__


def encode(obj):
    return json.dumps(obj, cls=Encoder, indent=4)


def saveDict(fileName, dataDict):
    fileHandle = open(fileName, 'w')
    json.dump(dataDict, fileHandle, cls=Encoder, indent=4)
    fileHandle.close()


def loadDict(fileName):
    fileHandle = open(fileName, 'r')
    fileDict = json.load(fileHandle)
    fileHandle.close()
    return fileDict

    
def save(fileName, params, scenario, vehicle):
    saveDict(fileName, {INPUT_PARAMS_KEY:params, SCENARIO_KEY:scenario, VEHICLE_KEY:vehicle})


def load(fileName):
    fileDict = loadDict(fileName)
    params = pathFindParams.fromDict(fileDict[INPUT_PARAMS_KEY])
    
    scenarioDict = fileDict[SCENARIO_KEY]
    
    noFlyZones = []
    for noFlyDict in scenarioDict["noFlyZones"]:
        noFlyZones.append(NoFlyZone(noFlyDict["points"], noFlyDict["velocity"], noFlyDict["ID"]))

    dynamicNoFlyZones = []
    for dNoFlyDict in scenarioDict["dynamicNoFlyZones"]:
        dynamicNoFlyZones.append(DynamicNoFlyZone(dNoFlyDict["center"], dNoFlyDict["radius"], dNoFlyDict["velocity"], dNoFlyDict["ID"]))

    roads = []
    for roadDict in scenarioDict["roads"]:
        roads.append(Road(roadDict["startPoint"], roadDict["endPoint"], roadDict["width"]))

    scenario = Scenario(scenarioDict["boundaryPoints"],
                                  noFlyZones,
                                  dynamicNoFlyZones,
                                  roads,
                                  scenarioDict["startPoint"],
                                  scenarioDict["startVelocity"],
                                  scenarioDict["wayPoints"])
        
    vehicle = Vehicle(fileDict[VEHICLE_KEY]["maxSpeed"],
                      fileDict[VEHICLE_KEY]["acceleration"])
    return (params, scenario, vehicle)

