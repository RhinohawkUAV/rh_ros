import editableNoFlyZone
from gui import Drawable


class EditableNoFlyZoneList(Drawable):

    def __init__(self, noFlyZones):
        self.noFlyZones = editableNoFlyZone.listFromInput(noFlyZones)

    def addNoFlyZone(self, editableNoFlyZone):
        self.noFlyZones.append(editableNoFlyZone)

    def addNoFlyZones(self, editableNoFlyZone):
        self.noFlyZones.extend(editableNoFlyZone)

    def removeNoFlyZone(self, editableNoFlyZone):
        self.noFlyZones.remove(editableNoFlyZone)

    def draw(self, visualizer, **kwargs):
        for editableNoFlyZone in self.noFlyZones:
            editableNoFlyZone.draw(visualizer, **kwargs)

    def findClosestPointIndex(self, point):
        closestDistanceSquared = float("inf")
        closestPointIndex = None
        closestNoFlyZone = None 

        for editableNoFlyZone in self.noFlyZones:
            (distSquared, NFZClosestPointIndex) = editableNoFlyZone.findClosestPoint(point)
            if distSquared < closestDistanceSquared:
                closestDistanceSquared = distSquared
                closestPointIndex = NFZClosestPointIndex
                closestNoFlyZone = editableNoFlyZone

        return (closestPointIndex, closestNoFlyZone)

    def findInsideNoFlyZones(self, point):
        noFlyZonesInside = []
        for editableNoFlyZone in self.noFlyZones:
            if editableNoFlyZone.isPointInside(point):
                noFlyZonesInside.append(editableNoFlyZone)
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

    def asInput(self):
        noFlyZones = []
        uniqueID = 0
        for editableNoFlyZone in self.noFlyZones:
            noFlyZones.append(editableNoFlyZone.asInput(uniqueID))
            uniqueID += 1
        return noFlyZones
