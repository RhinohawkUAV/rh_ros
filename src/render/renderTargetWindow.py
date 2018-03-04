import Tkinter as tk
import os
from Tkinter import Canvas
from Tkinter import Toplevel


class RenderTargetWindow(Toplevel):
    def __init__(self, canvasWidth, canvasHeight, viewCenterX, viewCenterY, viewWidth, viewHeight, **kw):
        Toplevel.__init__(self, **kw)
        self.viewCenterX = viewCenterX
        self.viewCenterY = viewCenterY
        self.viewWidth = viewWidth
        self.viewHeight = viewHeight
        self.title("Path Finding")
        self.canvas = Canvas(self, width=canvasWidth, height=canvasHeight)
        self.canvas.pack()
        self.protocol("WM_DELETE_WINDOW", self.close)

    def render(self, drawable):
        self.canvas.delete(tk.ALL)
        drawable.draw(self.canvas)
        self.transform()

    def transformCanvasToPoint(self, canvasPoint):
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        x = self.viewCenterX + self.viewWidth * (canvasPoint[0] - width / 2) / width
        y = self.viewCenterY - self.viewHeight * (canvasPoint[1] - height / 2) / height
        return (x, y)

    def transform(self):
        canvas = self.canvas
        width = canvas.winfo_width()
        height = canvas.winfo_height()

        canvas.move("all", -self.viewCenterX, -self.viewCenterY)
        canvas.scale("all", 0.0, 0.0, 2.0 / self.viewWidth, 2.0 / self.viewHeight)
        canvas.scale("all", 0.0, 0.0, width / 2.0, -height / 2.0)
        canvas.move("all", width / 2.0, height / 2.0)

    def close(self):
        os._exit(0)
