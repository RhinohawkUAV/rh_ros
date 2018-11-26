from constants import COURSE_DIM
import gui
from gui.editor import ScenarioEditor

if __name__ == '__main__':
    ScenarioEditor(800, 800, 0, 0, COURSE_DIM, COURSE_DIM)
    gui.startGUI()
