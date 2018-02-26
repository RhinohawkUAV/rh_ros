from Tkinter import Canvas

from typing import List

from render.Drawable import Drawable


class DrawGroup(Drawable):
    """A Drawable holding a group of drawables."""

    def __init__(self, drawables=None):
        # type: (List[Drawable])->None

        if drawables is None:
            drawables = []
        self.drawables = drawables

    def draw(self, canvas):
        # type: (Canvas)->None

        """Must be called from GUI thread"""
        for drawable in self.drawables:
            drawable.draw(canvas)

    def addDrawable(self, drawable):
        self.drawables.append(drawable)

    def clearDrawables(self):
        del self.drawables[:]
