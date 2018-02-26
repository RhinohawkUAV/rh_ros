from Tkinter import Canvas

from shapely.geometry import Polygon

from render.Drawable import Drawable


class DrawableLine(dict, Drawable):
    """Used to hold information about a line to draw on a canvas.
    It holds the coordinates of the line and is also a dictionary passed to the create_line() method."""

    def __init__(self, x1=None, y1=None, x2=None, y2=None, lineString=None, **kwargs):
        dict.__init__(self, **kwargs)
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

    def draw(self, canvas):
        # type: (Canvas)->None

        canvas.create_line(self.x1, self.y1, self.x2, self.y2, **self)


class DrawableCircle(dict, Drawable):
    """Used to hold information about a circle to draw on a canvas.
    It holds the coordinates of the circle and is also a dictionary passed to the create_oval() method."""

    def __init__(self, x=None, y=None, radius=None, **kwargs):
        dict.__init__(self, **kwargs)
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, canvas):
        # type: (Canvas)->None
        canvas.create_oval(self.x - self.radius, self.y - self.radius, self.x + self.radius, self.y + self.radius,
                           **self)


class DrawablePolygon(dict, Drawable):
    def __init__(self, points=None, polygon=None, **kwargs):
        dict.__init__(self, **kwargs)
        if polygon is None:
            self.polygon = Polygon(points)
        else:
            self.polygon = polygon

    def draw(self, canvas):
        # type: (Canvas)->None
        coords = []
        for vertex in self.polygon.exterior.coords:
            coords.append(vertex[0])
            coords.append(vertex[1])
        canvas.create_polygon(coords, **self)
