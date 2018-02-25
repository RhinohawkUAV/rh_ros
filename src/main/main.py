from sympy import Point2D

from geometry.geometry import Geometry
from geometry.noFlyZone import NoFlyZone
from render.renderer import Renderer

noFly1 = NoFlyZone([Point2D(10, 20), Point2D(30, 20), Point2D(30, 30), Point2D(10, 30)], Point2D(0, 0))
noFly2 = NoFlyZone([Point2D(40, 50), Point2D(40, 70), Point2D(50, 75), Point2D(50, 50)], Point2D(0, 0))

geo = Geometry([noFly1, noFly2])

geo.findVisibleVertices(Point2D(80, 80))

r = Renderer()

r.render(geo)
