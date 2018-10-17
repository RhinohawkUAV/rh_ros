
from Tkconstants import ARC
import math

import Tkinter as tk
import numpy as np

DEFAULT_POINT_SIZE = 5
DEFAULT_COLOR = "black"
DEFAULT_WIDTH = 1.0
DEFAULT_DASH = (1, 6)

# Multiply to convert velocity to pixels
VELOCITY_TO_PIXEL = 1.0


def drawPoint(visualizer, pos, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, outline=None, width=1.0, **kwargs):
    scaledRadius = visualizer.pixelVecToScale(np.array((radius, radius), np.double))
    visualizer.canvas.create_oval(pos[0] - scaledRadius[0], pos[1] - scaledRadius[1], pos[0] + scaledRadius[0], pos[1] + scaledRadius[1], fill=color, outline=outline, width=width)


def drawLine(visualizer, p1, p2, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, arrow=None, dash=None, **kwargs):
    visualizer.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=width, arrow=arrow, dash=dash)


def drawVelocity(visualizer, start, velocity, **kwargs):
    velocity = visualizer.pixelVecToScale(velocity * VELOCITY_TO_PIXEL)
    drawLine(visualizer, start, start + velocity, arrow=tk.LAST, **kwargs)


def drawPoly(visualizer, points, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, dash=None, **kwargs):
    for i in range(0, len(points)):
        drawLine(visualizer, points[i - 1], points[i], color=color, width=width, dash=dash, **kwargs)


def drawText(visualizer, position, text, color=DEFAULT_COLOR, offsetX=0.0, offsetY=0.0, **kwargs):
    offset = visualizer.pixelVecToScale(np.array((offsetX, offsetY), np.double))
    visualizer.canvas.create_text(position[0] + offset[0], position[1] + offset[1],
                       text=text,
                       fill=color, **kwargs)


def drawCircle(visualizer, center, radius, color=DEFAULT_COLOR, dash=None, **kwargs):
    visualizer.canvas.create_oval(center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius, outline=color, dash=dash, **kwargs)

    
def drawArc(visualizer, center, radius, startAngle, length, color=DEFAULT_COLOR, **kwargs):
    visualizer.canvas.create_arc(center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius, start=startAngle,
                      extent=length, outline=color, **kwargs)


def drawArcObj(visualizer, arc, **kwargs):
    drawArc(visualizer, arc.center, arc.radius,
            math.degrees(arc.start) * arc.rotDirection,
            math.degrees(arc.length) * arc.rotDirection,
            style=ARC, **kwargs)


def drawDynamicNoFlyZone(visualizer, dynamicNoFlyZone, color=DEFAULT_COLOR, time=0.0, **kwargs):
    center = dynamicNoFlyZone.center + dynamicNoFlyZone.velocity * time
    drawCircle(visualizer, center, dynamicNoFlyZone.radius, color=color)
    drawVelocity(visualizer, center, dynamicNoFlyZone.velocity)


def drawDynamicNoFlyZones(visualizer, dynamicNoFlyZones, color=DEFAULT_COLOR, **kwargs):
    for dnfz in dynamicNoFlyZones:
        drawDynamicNoFlyZone(visualizer, dnfz, color=color, **kwargs)


def drawNoFlyZone(visualizer, noFlyZone, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, time=0.0, **kwargs):
    for i in range(len(noFlyZone.points)):
        drawLine(visualizer,
                 noFlyZone.points[i - 1] + noFlyZone.velocity * time,
                 noFlyZone.points[i] + noFlyZone.velocity * time,
                 color=color, width=width)
    if np.linalg.norm(noFlyZone.velocity) > 0.0:
        midPoint = noFlyZone.points.sum(axis=0) / len(noFlyZone.points) + noFlyZone.velocity * time
        drawVelocity(visualizer, midPoint, noFlyZone.velocity, **kwargs)
    
        
def drawNoFlyZones(visualizer, noFlyZones, **kwargs):
        for noFlyZone in noFlyZones:
            drawNoFlyZone(visualizer, noFlyZone, **kwargs)


def drawWayPoints(visualizer, startPoint, startVelocity, wayPoints, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
    drawPoint(visualizer, startPoint, radius=radius, color=color)
    drawText(visualizer, startPoint, "Start", offsetY=15, color=color)
    drawVelocity(visualizer, startPoint,
                  startVelocity)

    for i in range(len(wayPoints)):
        point = wayPoints[i]
        drawPoint(visualizer, point, radius=radius, color=color)
        drawText(visualizer, point, "Waypoint: " + str(i), offsetY=15, color=color)


def drawScenario(visualizer, scenarioInput, time=0.0, **kwargs):
    drawNoFlyZones(visualizer, scenarioInput.noFlyZones, color="red", width=1.0, time=time, **kwargs)
    drawDynamicNoFlyZones(visualizer, scenarioInput.dynamicNoFlyZones, color="red", time=time, **kwargs)
    if len(scenarioInput.boundaryPoints) > 2:
        drawPoly(visualizer, scenarioInput.boundaryPoints, color="red")
    drawWayPoints(visualizer, scenarioInput.startPoint, scenarioInput.startVelocity, scenarioInput.wayPoints)

