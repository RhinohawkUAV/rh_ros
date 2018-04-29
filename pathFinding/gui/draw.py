import math

DEFAULT_POINT_SIZE = 0.5
DEFAULT_COLOR = "black"
DEFAULT_WIDTH = 1.0


def drawPoint(canvas, pos, radius=DEFAULT_POINT_SIZE, color=DEFAULT_COLOR, **kwargs):
    canvas.create_oval(pos[0] - radius, pos[1] - radius, pos[0] + radius, pos[1] + radius, fill=color)


def drawLine(canvas, p1, p2, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, arrow=None, **kwargs):
    canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=width, arrow=arrow)


def drawPoly(canvas, points, color=DEFAULT_COLOR, width=DEFAULT_WIDTH, **kwargs):
    for i in range(0, len(points)):
        drawLine(canvas, points[i - 1], points[i], color=color, width=width, **kwargs)


def drawText(canvas, position, text, color=DEFAULT_COLOR, **kwargs):
    canvas.create_text(position[0], position[1],
                       text=text,
                       fill=color, **kwargs)


def drawArc(canvas, center, radius, startAngle, endAngle):
    canvas.create_arc(center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius, start=startAngle,
                      extent=endAngle)


def drawArcObj(canvas, arc):
    drawArc(canvas, arc.center, arc.radius, math.degrees(arc.start), math.degrees(arc.length))
