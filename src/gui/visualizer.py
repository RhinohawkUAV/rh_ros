import Tkinter as tk
import os
from Tkinter import Canvas
from Tkinter import Toplevel

import core


class Visualizer(Toplevel, core.DrawListener):
    """
    Base class for windows used for visualization of problems.  Deals with many boilerplate TK tasks and has several features.
    - Registers blank listeners for common events and registers exit on close
    - Provides a "virtual coordinate system" that maps onto the window.  This is specified as a position and a width/height.
    - Provides reverse transform from mouse coordinates back to "virtual coordinate system"
    - Allows posting drawing into the GUI thread.  This can be overwhelmed if submissions are too fast.  This should leverage TK's dirty/repaint scheme in some way in the future.
    """

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
        self.bind('<Motion>', self.motion)
        self.bind('<Button-1>', self.leftClick)

    def drawInBackground(self, drawable, **kwargs):
        """Draw the given drawable in the GUI thread.
        drawable should not be touched while drawing."""
        core.inGUIThread(lambda: self.drawToCanvas(drawable, **kwargs))

    def drawToCanvas(self, drawable, **kwargs):
        """Draw the given drawable, applying virtual coordinates transform.
        Must be called from GUI THREAD!"""
        self.canvas.delete(tk.ALL)
        drawable.draw(self.canvas, **kwargs)
        self.transformCanvas()

    def transformCanvas(self):
        """Transforms all objects drawn on the canvas in the "virtual coordinate system" to the window's coordinates."""
        canvas = self.canvas
        width = canvas.winfo_width()
        height = canvas.winfo_height()

        canvas.move("all", -self.viewCenterX, -self.viewCenterY)
        canvas.scale("all", 0.0, 0.0, 2.0 / self.viewWidth, 2.0 / self.viewHeight)
        canvas.scale("all", 0.0, 0.0, width / 2.0, -height / 2.0)
        canvas.move("all", width / 2.0, height / 2.0)

    def transformCanvasToPoint(self, canvasPoint):
        """
        Given a point on the canvas, such as a mouse coordinate, transform it to the "virtual coordinate system".
        :param canvasPoint: point to transform
        :return: (transX, transY)
        """
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        transX = self.viewCenterX + self.viewWidth * (canvasPoint[0] - width / 2.0) / width
        transY = self.viewCenterY - self.viewHeight * (canvasPoint[1] - height / 2.0) / height
        return (transX, transY)

    def onDraw(self, drawable, **kwargs):
        """Callback for DrawListener.  This can be used by a background calculation thread to signal the GUI to draw a new state."""
        self.drawInBackground(drawable)

    def leftClick(self, event):
        pass

    def motion(self, event):
        pass

    def close(self):
        os._exit(0)
