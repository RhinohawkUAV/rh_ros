from geometry import generator
from render.renderer import Renderer

renderer = Renderer()

# noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (0, 0))
# noFly2 = NoFlyZone([(40, 40), (40, 45), (45, 45), (45, 40)], (0, 0))
# noFly3 = NoFlyZone([(10, 20), (30, 20), (30, 30), (10, 30)], (0, 0))
#
# geo = Geometry([noFly1, noFly2, noFly3])

geo = generator.generateGeo(25, 10, 10, 80, 80, 0.01, 0.1)
start = (95, 95)
end = (5, 5)
geo.findPath(start, end, renderer)
