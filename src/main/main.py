import copy
import random
import time

from geometry.geometry import Geometry
from geometry.noFlyZone import NoFlyZone
from render.renderer import Renderer

r = Renderer()

noFly1 = NoFlyZone([(10, 20), (30, 20), (30, 30), (10, 30)], (0, 0))
noFly2 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
noFly3 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (0, 0))

geo = Geometry([noFly1, noFly2, noFly3])

position = (80, 80)
while True:
    verts = geo.findVisibleVertices(position)
    renderCopy = copy.deepcopy(geo)
    r.render(renderCopy)
    index = random.randint(0, len(verts) - 1)
    position = verts[index]
    geo.clearDrawables()
    time.sleep(3)
