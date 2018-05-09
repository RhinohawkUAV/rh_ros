"""
Utilities related to loading/saving/generating scenarios.
"""
import json
import numpy as np
from engine.interface.vehicleInput import VehicleInput
from engine.interface.noFlyZoneInput import NoFlyZoneInput
from engine.interface.debugInput import DebugInput
from engine.interface.scenarioInput import ScenarioInput
from engine.interface.testInput import TestInput



class Encoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif callable(getattr(obj, "toJSONDict", None)):
            return obj.toJSONDict()
        else:
            return obj.__dict__


# def saveScenario(fileName, scenarioInput, vehicleInput, testInput):
#     fileHandle = open(fileName, 'w')
#     output = {}
#     output[OBSTACLE_INPUT_KEY] = obstacleCourseEdit
#     output[PATH_INPUT_KEY] = pathEdit
#     if extraDict is not None:
#         output.update(extraDict)
#     json.dump(output, fileHandle, cls=Encoder, indent=4)
#     fileHandle.close()


def loadInputAsDict(fileName):
    fileHandle = open(fileName, 'r')
    fileDict = json.load(fileHandle)
    fileHandle.close()
    
    output = {}
    
    scenarioDict = fileDict["scenario"]
    
    noFlyZones = []
    for noFlyDict in scenarioDict["noFlyZones"]:
        noFlyZones.append(NoFlyZoneInput(noFlyDict["points"],noFlyDict["velocity"],noFlyDict["ID"]))


    scenarioInput = ScenarioInput(scenarioDict["boundaryPoints"],
                                  scenarioDict["noFlyZones"], 
                                  scenarioDict["roads"], 
                                  scenarioDict["startPoint"], 
                                  scenarioDict["startVelocity"], 
                                  scenarioDict["wayPoints"])
    
    output["scenario"] = scenarioInput

    vehicleInput = VehicleInput(fileDict["vehicle"]["acceleration"])
    output["vehicle"] = vehicleInput

    if fileDict.has_key("testInput"):
        testDict = fileDict["testInput"]
        testInput = TestInput(testDict["startPoint"], 
                                     testDict["startVelocity"], 
                                     testDict["targetPoint"], 
                                     testDict["velocityOfTarget"])
        output["testInput"] = testInput
    return output

def loadInput(fileName):
    inputDict = loadInputAsDict(fileName)
    
    if inputDict.has_key("testInput"):
        return DebugInput(inputDict["scenario"],
                          inputDict["vehicle"],
                          inputDict["testInput"])
    else:
        return ScenarioInput(inputDict["scenario"],
                             inputDict["vehicle"])

def loadInputDebug(fileName):
    inputDict = loadInputAsDict(fileName)
    if inputDict.has_key("testInput"):
        testInput = inputDict["testInput"]
    else:
        testInput = TestInput((5.0,5.0),(1.0,1.0),(95.0,95.0),(0.0,0.0))
        return DebugInput(inputDict["scenario"],
                          inputDict["vehicle"],
                          testInput)

    