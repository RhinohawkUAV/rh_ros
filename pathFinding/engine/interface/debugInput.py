from engine.interface.input import Input
from engine.interface.vehicleInput import VehicleInput
from engine.interface import testInput, scenarioInput
class DebugInput(Input):
    def __init__(self,scenarioInput, vehicleInput, testInput):
        Input.__init__(self,scenarioInput,vehicleInput)
        self.testInput = testInput

def defaultValue():
    scenario = scenarioInput.defaultValue()
    vehicle = VehicleInput(1.0)
    test = testInput.defaultValue()
    return DebugInput(scenario,vehicle,test)