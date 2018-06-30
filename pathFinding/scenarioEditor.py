# TODO: Remove
# start = np.array([0.7916, -0.61], np.double)
# end = np.array([-0.83, 0.557], np.double)
#
# start = np.array([math.sqrt(3) / 2, -0.5], np.double)
# end = np.array([-math.sqrt(3) / 2, 0.5], np.double)
# end = calcs.rotate2d(end, -0.01)
#
# print arcObstacleData.relativeAngleCCW(start, end)
#
from constants import COURSE_DIM
import gui
from gui.editor import ScenarioEditor

ScenarioEditor(800, 800, 50, 50, COURSE_DIM, COURSE_DIM)

gui.startGUI()
