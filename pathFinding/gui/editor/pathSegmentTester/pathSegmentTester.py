from engine.geometry import calcs
from engine.geometry.obstacle import obstacleCourse
from engine.interface.fileUtils import TEST_INPUT_KEY, SCENARIO_KEY
from engine.interface.outputPath import OutputPath
from engine.interface.pathFindParams import DEFAULT_PARAMS
from engine.interface.vehicle import DEFAULT_VEHICLE
from gui import draw
from gui.draw import DEFAULT_COLOR, DEFAULT_POINT_SIZE, VELOCITY_SCALE
from gui.editor.subGUI import SubGUI
from gui.pathFinder.pathfindDrawable import PathFindDrawable


class PathSegmentTester(SubGUI):

    def __init__(self):
        self._pathFinderDrawable = None
        self._obstacleCourse = None
        self._pointOfInterest = None
        self._showPathsToPoints = False

    def onLeftRelease(self, point, control=False):
        if control:
            self._inputDict[TEST_INPUT_KEY].targetPoint = point
        else:
            self._inputDict[TEST_INPUT_KEY].startPoint = point
            
        self.updateDrawable()

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if ctrl:
                self._inputDict[TEST_INPUT_KEY].velocityOfTarget = (point - self._inputDict[TEST_INPUT_KEY].targetPoint) / VELOCITY_SCALE
            else:
                self._inputDict[TEST_INPUT_KEY].startVelocity = (point - self._inputDict[TEST_INPUT_KEY].startPoint) / VELOCITY_SCALE

        if key == "z":
            self._showPathsToPoints = not self._showPathsToPoints
        self.updateDrawable()

    def onMotion(self, point, control=False):
        self._pointOfInterest = point

    def onSwitch(self, inputDict):
        SubGUI.onSwitch(self, inputDict)
        self._obstacleCourse = obstacleCourse.createObstacleCourse(DEFAULT_PARAMS, DEFAULT_VEHICLE, self._inputDict[SCENARIO_KEY])
        self._pathFinderDrawable = PathFindDrawable(DEFAULT_PARAMS, DEFAULT_VEHICLE, self._inputDict[SCENARIO_KEY])

    def updateDrawable(self):
        (startUnitVelocity, startSpeed) = calcs.unitAndLength(self._inputDict[TEST_INPUT_KEY].startVelocity)
                 
        (goalSegments, filteredGoalSegments) = self._obstacleCourse.findPathSegmentsToPoint(startTime=0.0,
                                                         startPoint=self._inputDict[TEST_INPUT_KEY].startPoint,
                                                         startSpeed=startSpeed,
                                                         startUnitVelocity=startUnitVelocity,
                                                         targetPoint=self._inputDict[TEST_INPUT_KEY].targetPoint,
                                                         velocityOfTarget=self._inputDict[TEST_INPUT_KEY].velocityOfTarget,
                                                         legalRotDirection=0.0)

        if self._showPathsToPoints:
            (pathSegments, filteredPathSegments) = self._obstacleCourse.findPathSegments(startTime=0.0,
                                                                                                   startPoint=self._inputDict[TEST_INPUT_KEY].startPoint,
                                                                                                   startSpeed=startSpeed,
                                                                                                   startUnitVelocity=startUnitVelocity,
                                                                                                   legalRotDirection=0.0)
        else:
            pathSegments = []
            filteredPathSegments = []
        filteredPathSegments.extend(filteredGoalSegments)
        
        self._pathFinderDrawable.update(False, OutputPath([], goalSegments, None, 0.0), [], pathSegments, filteredPathSegments)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        drawTime = self._pathFinderDrawable.draw(canvas,
                                      pointOfInterest=self._pointOfInterest,
                                      snapDistance=50.0,
                                      showFiltered=True)
        draw.drawPoint(canvas, self._inputDict[TEST_INPUT_KEY].startPoint, radius=radius, color=color)
        draw.drawVelocity(canvas,
                          self._inputDict[TEST_INPUT_KEY].startPoint,
                          self._inputDict[TEST_INPUT_KEY].startVelocity)

        targetPoint = self._inputDict[TEST_INPUT_KEY].targetPoint + self._inputDict[TEST_INPUT_KEY].velocityOfTarget * drawTime
        draw.drawPoint(canvas, targetPoint, radius=radius, color=color)
        draw.drawVelocity(canvas, targetPoint, self._inputDict[TEST_INPUT_KEY].velocityOfTarget)

