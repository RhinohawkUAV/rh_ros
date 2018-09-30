"""
Utilities related to loading/saving/generating scenarios.
"""
import json

from engine.interface import testScenario
from engine.interface.dynamicNoFlyZone import DynamicNoFlyZone
from engine.interface.noFlyZone import NoFlyZone
from engine.interface.road import Road
from engine.interface.scenario import Scenario
from engine.interface.testScenario import TestScenario
import numpy as np

INPUT_PARAMS_KEY = "params"
SCENARIO_KEY = "scenario"
VEHICLE_KEY = "vehicle"
TEST_INPUT_KEY = "testScenario"


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


def save(fileName, params, scenario, vehicle):
    fileHandle = open(fileName, 'w')
    json.dump({INPUT_PARAMS_KEY:params, SCENARIO_KEY:scenario, VEHICLE_KEY:vehicle}, fileHandle, cls=Encoder, indent=4)
    fileHandle.close()


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
        noFlyZones.append(NoFlyZone(noFlyDict["points"], noFlyDict["velocity"], noFlyDict["ID"]))

    dynamicNoFlyZones = []
    for dNoFlyDict in scenarioDict["dynamicNoFlyZones"]:
        dynamicNoFlyZones.append(DynamicNoFlyZone(dNoFlyDict["center"], dNoFlyDict["radius"], dNoFlyDict["velocity"], dNoFlyDict["ID"]))

    roads = []
    for roadDict in scenarioDict["roads"]:
        roads.append(Road(roadDict["startPoint"], roadDict["endPoint"], roadDict["width"]))

    scenarioInput = Scenario(scenarioDict["boundaryPoints"],
                                  noFlyZones,
                                  dynamicNoFlyZones,
                                  roads,
                                  scenarioDict["startPoint"],
                                  scenarioDict["startVelocity"],
                                  scenarioDict["wayPoints"])
    
    inputDict[SCENARIO_KEY] = scenarioInput

    if fileDict.has_key(TEST_INPUT_KEY):
        testDict = fileDict[TEST_INPUT_KEY]
        testScenario = TestScenario(testDict["startPoint"],
                                     testDict["startVelocity"],
                                     testDict["targetPoint"],
                                     testDict["velocityOfTarget"])
        inputDict[TEST_INPUT_KEY] = testScenario
    return inputDict
    
