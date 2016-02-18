import numpy as np
import sys
from ui_structures import *

# Create window and set backing
window = DisplayWindow(1366, 768, resizable=True)
window.set_minimum_size(854, 480)
window.set_maximum_size(2560, 1440)
pyglet.gl.glClearColor(0.23529412,  0.24705882,  0.24705882, 1.0)

# Create icon sets
icon_16 = pyglet.image.load('./resources/icon_16.png')
icon_32 = pyglet.image.load('./resources/icon_32.png')
icon_64 = pyglet.image.load('./resources/icon_64.png')
icon_128 = pyglet.image.load('./resources/icon_128.png')
icon_256 = pyglet.image.load('./resources/icon_256.png')
window.set_icon(icon_16, icon_32, icon_64, icon_128, icon_256)
window.set_caption("PROJECT HUMMINGBIRD Real-time Visualization Plotter")

print(window.__hash__())

def update(dt=0):
    global message_string
    message_string = "%.3f" % time.time()

window.schedule = pyglet.clock.schedule_interval(update, 1 / 45.0)  # update at 60Hz
pyglet.app.run()

# TODO formula for small-text scaling: 11.0/1366.0 + SCREEN_WIDTH/256.0
# TODO 38400 baudrate corresponds with ~x-xx ms time to receive next data-set according to rather simplistic calculation