import Tkinter as tk
from threading import Thread

from render.renderWindow import RenderWindow


class Renderer:
    def __init__(self):
        # Create TK root object and hide it
        self.root = tk.Tk()  # type: tk.Tk
        self.root.withdraw()
        self.renderWindow = RenderWindow(800, 800, 50, 50, 100, 100)

        # Create/start the GUIThread
        self.guiThread = RenderThread(self.root)
        self.guiThread.start()

    def inGUIThread(self, task, *args):
        """Run a task in the GUI thread"""
        self.root.after_idle(task, *args)

    def render(self,
               drawable
               ):
        """Renders a drawable object in the GUI thread"""
        self.inGUIThread(self._render, drawable)

    def _render(self, drawable):
        """Renders drawable object and then transforms to canvas coordinates"""
        self.renderWindow.canvas.delete(tk.ALL)
        drawable.draw(self.renderWindow.canvas)
        self.renderWindow.transform()


class RenderThread(Thread):
    def __init__(self, root):
        Thread.__init__(self, name="GUI Thread")
        self.root = root

    def run(self):
        self.root.mainloop()
