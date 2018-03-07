from Tkinter import Canvas

import numpy as np

from gui import Drawable


class DrawableLine(Drawable):
    def __init__(self, x1=None, y1=None, x2=None, y2=None, lineString=None):
        if lineString is None:
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2
        else:
            self.x1 = float(lineString.coords[0][0])
            self.y1 = float(lineString.coords[0][1])
            self.x2 = float(lineString.coords[1][0])
            self.y2 = float(lineString.coords[1][1])

    def draw(self, canvas, text="", **kwargs):
        # type: (Canvas)->None

        if not text == "":
            midPoint = np.array([(self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2])
            textOffset = np.array([-self.y2 - self.y1, self.x2 - self.x1])
            textOffset = 4 * textOffset / np.linalg.norm(textOffset, 2)
            textPos = midPoint + textOffset
            canvas.create_text(textPos[0], textPos[1], text=text, fill="black")
        canvas.create_line(self.x1, self.y1, self.x2, self.y2, **kwargs)


class DrawableCircle(Drawable):
    def __init__(self, x=None, y=None, radius=None):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, canvas, **kwargs):
        # type: (Canvas)->None
        canvas.create_oval(self.x - self.radius, self.y - self.radius, self.x + self.radius, self.y + self.radius,
                           **kwargs)


class DrawablePolygon(Drawable):
    def __init__(self, points):
        self.points = points

    def draw(self, canvas, **kwargs):
        # type: (Canvas)->None
        coords = []
        for point in self.points:
            coords.append(point[0])
            coords.append(point[1])
        canvas.create_polygon(coords, **kwargs)
