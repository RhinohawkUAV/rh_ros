import Tkinter as tk

root = tk.Tk()
root.withdraw()


def startGUI():
    global root
    root.mainloop()


def inGUIThread(task, *args):
    """Run a task in the GUI thread"""
    global root
    root.after_idle(task, *args)


class Drawable:
    """Something that can be draw itself on the given TK visualizer object."""

    def draw(self, visualizer, **kwargs):
        """
        Draw on the visualizer with any modifiers stored in kwargs.
        :param visualizer:
        :param kwargs:
        :return:
        """
        # type: (visualizer) -> None
        pass


class DrawListener:
    """Used for computations threads that want to communicate back to the GUI, on a regular basis, with a new state to draw."""

    def onDraw(self, drawable, **kwargs):
        pass
