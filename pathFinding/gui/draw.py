
from Tkconstants import ARC
import math

import Tkinter as tk
import numpy as np

# TODO: Offset will not look correct for other scalings, similar problem for DEFAULT_POINT_SIZE.
# Ideally this will be a non-scaling factor.
TEXT_OFFSET = np.array((0, -2.0), np.double)

DEFAULT_POINT_SIZE = 0.5
DEFAULT_COLOR = "black"
DEFAULT_WIDTH = 1.0
DEFAULT_DASH = (1, 6)


def drawPoint(canvas, pos, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
    canvas.create_oval(pos[0] - radius, pos[1] - radius, pos[0] + radius, pos[1] + radius, fill=color)


def drawLine(canvas, p1, p2, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, arrow=None, dash=None, **kwargs):
    canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=width, arrow=arrow, dash=dash)


def drawPoly(canvas, points, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, dash=None, **kwargs):
    for i in range(0, len(points)):
        drawLine(canvas, points[i - 1], points[i], color=color, width=width, dash=dash, **kwargs)


def drawText(canvas, position, text, color=DEFAULT_COLOR, **kwargs):
    canvas.create_text(position[0], position[1],
                       text=text,
                       fill=color, **kwargs)


def drawArc(canvas, center, radius, startAngle, endAngle, color=DEFAULT_COLOR, **kwargs):
    canvas.create_arc(center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius, start=startAngle,
                      extent=endAngle, outline=color, **kwargs)


def drawArcObj(canvas, arc, **kwargs):
    drawArc(canvas, arc.center, arc.radius, math.degrees(arc.start) * arc.direction,
            math.degrees(arc.length) * arc.direction,
            style=ARC, **kwargs)


def drawNoFlyZone(canvas, noFlyZone, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, **kwargs):
    for i in range(len(noFlyZone.points)):
        drawLine(canvas, noFlyZone.points[i - 1], noFlyZone.points[i], color=color, width=width)
    if np.linalg.norm(noFlyZone.velocity) > 0.0:
        midPoint = noFlyZone.points.sum(axis=0) / len(noFlyZone.points)
        drawLine(canvas, midPoint, midPoint + noFlyZone.velocity, arrow=tk.LAST, **kwargs)
    
        
def drawNoFlyZones(canvas, noFlyZones, **kwargs):
        for noFlyZone in noFlyZones:
            drawNoFlyZone(canvas, noFlyZone, **kwargs)


def drawWayPoints(canvas, startPoint, startVelocity, wayPoints, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
    drawPoint(canvas, startPoint, radius=radius, color=color)
    drawText(canvas, startPoint + TEXT_OFFSET, "Start", color=color)
    drawLine(canvas, startPoint,
                  startPoint + startVelocity,
                  arrow=tk.LAST)

    for i in range(len(wayPoints)):
        point = wayPoints[i]
        drawPoint(canvas, point, radius=radius, color=color)
        drawText(canvas, point + TEXT_OFFSET, "Waypoint: " + str(i), color=color)


def drawScenario(canvas, scenarioInput, **kwargs):
    drawNoFlyZones(canvas, scenarioInput.noFlyZones, color="black", width=1.0, **kwargs)
    if len(scenarioInput.boundaryPoints) > 2:
        drawPoly(canvas, scenarioInput.boundaryPoints, color="red")
    drawWayPoints(canvas, scenarioInput.startPoint, scenarioInput.startVelocity, scenarioInput.wayPoints)


def drawInput(canvas, debugInput, **kwargs):
    drawScenario(canvas, debugInput.scenario, **kwargs)

