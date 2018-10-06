from engine.geometry import calcs
from engine.geometry.obstacle import obstacleCourse
from engine.interface.outputPath import OutputPath
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
            self._testInput.targetPoint = point
        else:
            self._testInput.startPoint = point
            
        self.updateDrawable()

    def onKey(self, point, key, ctrl=False):
        if key == "v":
            if ctrl:
                self._testInput.velocityOfTarget = (point - self._testInput.targetPoint) / VELOCITY_SCALE
            else:
                self._testInput.startVelocity = (point - self._testInput.startPoint) / VELOCITY_SCALE

        if key == "z":
            self._showPathsToPoints = not self._showPathsToPoints
        self.updateDrawable()

    def onMotion(self, point, control=False):
        self._pointOfInterest = point

    def onSwitch(self, params, scenario, vehicle, testInput):
        SubGUI.onSwitch(self, params, scenario, vehicle, testInput)
        self._obstacleCourse = obstacleCourse.createObstacleCourse(self._params, self._vehicle, self._scenario)
        self._pathFinderDrawable = PathFindDrawable(self._params, self._vehicle, self._scenario)

    def updateDrawable(self):
        (startUnitVelocity, startSpeed) = calcs.unitAndLength(self._testInput.startVelocity)
                 
        (goalSegments, filteredGoalSegments) = self._obstacleCourse.findPathSegmentsToPoint(startTime=0.0,
                                                         startPoint=self._testInput.startPoint,
                                                         startSpeed=startSpeed,
                                                         startUnitVelocity=startUnitVelocity,
                                                         targetPoint=self._testInput.targetPoint,
                                                         velocityOfTarget=self._testInput.velocityOfTarget,
                                                         legalRotDirection=0.0)

        if self._showPathsToPoints:
            (pathSegments, filteredPathSegments) = self._obstacleCourse.findPathSegments(startTime=0.0,
                                                                                                   startPoint=self._testInput.startPoint,
                                                                                                   startSpeed=startSpeed,
                                                                                                   startUnitVelocity=startUnitVelocity,
                                                                                                   legalRotDirection=0.0)
        else:
            pathSegments = []
            filteredPathSegments = []
        filteredPathSegments.extend(filteredGoalSegments)
        
        self._pathFinderDrawable.update(False, OutputPath([], goalSegments, 3, 0, 0.0), [], pathSegments, filteredPathSegments)

    def draw(self, canvas, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
        drawTime = self._pathFinderDrawable.draw(canvas,
                                      pointOfInterest=self._pointOfInterest,
                                      snapDistance=50.0,
                                      showFiltered=True)
        draw.drawPoint(canvas, self._testInput.startPoint, radius=radius, color=color)
        draw.drawVelocity(canvas,
                          self._testInput.startPoint,
                          self._testInput.startVelocity)

        targetPoint = self._testInput.targetPoint + self._testInput.velocityOfTarget * drawTime
        draw.drawPoint(canvas, targetPoint, radius=radius, color=color)
        draw.drawVelocity(canvas, targetPoint, self._testInput.velocityOfTarget)

