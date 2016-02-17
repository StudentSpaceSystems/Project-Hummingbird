import pyglet
from pyglet.gl import *
import random

w, x, y, z = random.randint(1,100), random.randint(1,100),\
             random.randint(1,1100), random.randint(1,500)

window = pyglet.window.Window(1280,600)

batch = pyglet.graphics.Batch()

size = window.get_size()

@window.event
def on_draw():
    window.clear()
    pyglet.graphics.draw(2, pyglet.gl.GL_LINES, \
                         ('v2f',\
                          (w, x, y, z)))
    pyglet.graphics.draw_indexed(4, pyglet.gl.GL_TRIANGLES,
    [0, 1, 2, 0, 2, 3],
    ('v2i', (100, 100,
             150, 100,
             150, 150,
             100, 150)))
    vertex_list = pyglet.graphics.vertex_list(3,
    ('v2i', (10, 15, 30, 35, 25, 60)),
    ('c3B', (0, 0, 255, 0, 255, 0, 255, 0, 0)))
    vertex_list.draw(pyglet.gl.GL_TRIANGLES)


def update(dt):
    global w, x, y, z
    w, x, y, z = random.randint(1,100), random.randint(1,100),\
                 random.randint(1,1100), random.randint(1,500)

window.schedule = pyglet.clock.schedule_interval(update, 1/60.0) # update at 60Hz
pyglet.app.run()