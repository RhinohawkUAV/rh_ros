import sys
from Tkinter import Canvas
from Tkinter import Toplevel


class RenderWindow(Toplevel):
    def __init__(self, canvasWidth, canvasHeight, viewCenterX, viewCenterY, viewWidth, viewHeight, **kw):
        Toplevel.__init__(self, **kw)
        self.viewCenterX = viewCenterX
        self.viewCenterY = viewCenterY
        self.viewWidth = viewWidth
        self.viewHeight = viewHeight
        self.title("Path Finding")
        self.canvas = Canvas(self, width=canvasWidth, height=canvasHeight)
        self.canvas.pack()
        self.protocol("WM_DELETE_WINDOW", sys.exit)

    def transform(self):
        canvas = self.canvas
        width = canvas.winfo_width()
        height = canvas.winfo_height()

        canvas.move("all", -self.viewCenterX, -self.viewCenterY)
        canvas.scale("all", 0.0, 0.0, 2.0 / self.viewWidth, 2.0 / self.viewHeight)
        canvas.scale("all", 0.0, 0.0, width / 2.0, -height / 2.0)
        canvas.move("all", width / 2.0, height / 2.0)
