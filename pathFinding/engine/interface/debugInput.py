from engine.interface.input import Input
class DebugInput(Input):
    def __init__(self,scenarioInput, vehicleInput, testInput):
        Input.__init__(self,scenarioInput,vehicleInput)
        self.testInput = testInput