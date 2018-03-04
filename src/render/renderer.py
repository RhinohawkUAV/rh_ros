import Tkinter as tk
from threading import Thread


class Renderer:
    def __init__(self):
        # Create TK root object and hide it
        self.root = tk.Tk()  # type: tk.Tk
        self.root.withdraw()
        self._renderTarget = None

        # Create/start the GUIThread
        self.guiThread = GUIThread(self.root)
        self.guiThread.start()

    def setRenderTarget(self, renderTarget):
        """Sets the render output target.  Can ONLY BE CALLED IN GUI THREAD!"""
        self._renderTarget = renderTarget

    def inGUIThread(self, task, *args):
        """Run a task in the GUI thread"""
        self.root.after_idle(task, *args)

    def render(self, drawable):
        """Renders a drawable object in the GUI thread"""
        self.inGUIThread(self._render, drawable)

    def _render(self, drawable):
        """Renders drawable object and then transforms to canvas coordinates"""
        return self._renderTarget.render(drawable)


class GUIThread(Thread):
    def __init__(self, root):
        Thread.__init__(self, name="GUI Thread")
        self.root = root

    def run(self):
        self.root.mainloop()
