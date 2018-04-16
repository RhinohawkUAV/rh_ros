from engine import InitialPathFindingInput
from gui import Drawable, draw
from gui.editor import noFlyZoneEdit


class InitialPathFindingEdit(InitialPathFindingInput, Drawable):
    def __init__(self):
        InitialPathFindingInput.__init__(self, [], [])

    def setToInput(self, input):
        self.boundaryPoints = input.boundaryPoints
        self.noFlyZones = noFlyZoneEdit.listFromInput(input.noFlyZones)

    def addNoFlyZone(self, noFlyZoneEdit):
        self.noFlyZones.append(noFlyZoneEdit)

    def addNoFlyZones(self, noFlyZoneEdit):
        self.noFlyZones.extend(noFlyZoneEdit)

    def removeNoFlyZone(self, noFlyZoneEdit):
        self.noFlyZones.remove(noFlyZoneEdit)

    def draw(self, canvas, **kwargs):
        for noFlyZoneEdit in self.noFlyZones:
            noFlyZoneEdit.draw(canvas, **kwargs)

        if len(self.boundaryPoints) > 2:
            draw.drawPoly(canvas, self.boundaryPoints, color="red")

    def findClosestPointIndex(self, point):
        closestDistanceSquared = float("inf")
        closestPointIndex = None
        closestNoFlyZone = None

        for noFlyZoneEdit in self.noFlyZones:
            (distSquared, NFZClosestPointIndex) = noFlyZoneEdit.findClosestPoint(point)
            if distSquared < closestDistanceSquared:
                closestDistanceSquared = distSquared
                closestPointIndex = NFZClosestPointIndex
                closestNoFlyZone = noFlyZoneEdit

        return (closestPointIndex, closestNoFlyZone)

    def findInsideNoFlyZones(self, point):
        noFlyZonesInside = []
        for noFlyZoneEdit in self.noFlyZones:
            if noFlyZoneEdit.isPointInside(point):
                noFlyZonesInside.append(noFlyZoneEdit)
        return noFlyZonesInside

    def removeInsideNoFlyZones(self, point):
        noFlyZonesInside = []
        i = 0
        while i < len(self.noFlyZones):
            if self.noFlyZones[i].isPointInside(point):
                noFlyZonesInside.append(self.noFlyZones[i])
                del self.noFlyZones[i]
            else:
                i += 1
        return noFlyZonesInside

    def toJSONDict(self):
        noFlyZoneInputs = []
        uniqueID = 0
        for noFlyZoneEdit in self.noFlyZones:
            noFlyZoneInputs.append(noFlyZoneEdit.toInput(uniqueID))
            uniqueID += 1
        return InitialPathFindingInput(self.boundaryPoints, noFlyZoneInputs).__dict__
