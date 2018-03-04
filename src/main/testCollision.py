from geometry.noFlyZone import NoFlyZone
from render.collisionRenderTargetWindow import CollisionRenderTargetWindow
from render.renderer import Renderer


def createWindow():
    window = CollisionRenderTargetWindow(800, 800, 50, 50, 100, 100)

    window.startPoint = (10, 10)
    window.speed = 5.0

    noFly1 = NoFlyZone([(40, 50), (40, 70), (50, 75), (50, 50)], (2, 0))
    window.noFlyZones = [noFly1]

    renderer.setRenderTarget(window)


renderer = Renderer()
renderer.inGUIThread(createWindow)
