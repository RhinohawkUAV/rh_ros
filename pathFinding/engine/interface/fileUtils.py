"""
Utilities related to loading/saving/generating scenarios.
"""
import json

from engine.interface import testInput
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZoneInput
from engine.interface.noFlyZoneInput import NoFlyZoneInput
from engine.interface.roadInput import RoadInput
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.testInput import TestInput
from engine.interface.vehicleInput import VehicleInput
import numpy as np

INPUT_PARAMS_KEY = "params"
SCENARIO_KEY = "scenario"
TEST_INPUT_KEY = "testInput"


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


def saveInput(fileName, inputDict):
    fileHandle = open(fileName, 'w')
    json.dump(inputDict, fileHandle, cls=Encoder, indent=4)
    fileHandle.close()


def loadInput(fileName):
    fileHandle = open(fileName, 'r')
    fileDict = json.load(fileHandle)
    fileHandle.close()
    
    inputDict = {}
    
    scenarioDict = fileDict[SCENARIO_KEY]
    
    noFlyZones = []
    for noFlyDict in scenarioDict["noFlyZones"]:
        noFlyZones.append(NoFlyZoneInput(noFlyDict["points"], noFlyDict["velocity"], noFlyDict["ID"]))

    dynamicNoFlyZones = []
    for dNoFlyDict in scenarioDict["dynamicNoFlyZones"]:
        dynamicNoFlyZones.append(DynamicNoFlyZoneInput(dNoFlyDict["center"], dNoFlyDict["radius"], dNoFlyDict["velocity"], dNoFlyDict["ID"]))

    roads = []
    for roadDict in scenarioDict["roads"]:
        roads.append(RoadInput(roadDict["startPoint"], roadDict["endPoint"], roadDict["width"]))

    scenarioInput = ScenarioInput(scenarioDict["boundaryPoints"],
                                  noFlyZones,
                                  dynamicNoFlyZones,
                                  roads,
                                  scenarioDict["startPoint"],
                                  scenarioDict["startVelocity"],
                                  scenarioDict["wayPoints"])
    
    inputDict[SCENARIO_KEY] = scenarioInput

    if fileDict.has_key(TEST_INPUT_KEY):
        testDict = fileDict[TEST_INPUT_KEY]
        testInput = TestInput(testDict["startPoint"],
                                     testDict["startVelocity"],
                                     testDict["targetPoint"],
                                     testDict["velocityOfTarget"])
        inputDict[TEST_INPUT_KEY] = testInput
    return inputDict
    
