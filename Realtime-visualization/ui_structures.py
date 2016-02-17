import pyglet
from pyglet.gl import *
from pyglet.window import key, mouse
import time


class DisplayWindow(pyglet.window.Window):

    def init_structures(self, *args, **kwargs):
        self.batch = pyglet.graphics.Batch()
        self.children = []
        self.visible_children = {}

    def add_child(self, child):
        self.children.append(child)
        self.visible_children[()]

    def on_key_press(self, symbol, modifier):
        # Refer to BaseWindow implementation of on_key_press if you find disagreement with syntax choice
        if symbol == key.ESCAPE and not\
                (modifier & ~(key.MOD_NUMLOCK | key.MOD_CAPSLOCK | key.MOD_SCROLLLOCK)):

            self.dispatch_event('on_close')

    def on_draw(self):
        self.clear()
        message_string = "%.3f" % time.time()
        reset_text = pyglet.text.Label(message_string, \
                                       font_name='Times New Roman', \
                                       font_size=36, \
                                       color=(255, 255, 255, 255), \
                                       x=self.width / 2, \
                                       y=self.height / 2, \
                                       anchor_x='center', anchor_y='center')
        reset_text.draw()

    def on_mouse_press(self, x, y, button, modifiers):
        pass

    def on_mouse_release(self, x, y, button, modifiers):
        pass

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        pass

class Interface:

    def __init__(self, x_i, y_i, x_f, y_f, screen_width, screen_height, ):
        self.visible = False
        self.has_focus = False
        self.draw_instructions = []
        self.children = []
        self.visible_children = {}

    def set_visible(self, visibility):
        self.visible = visibility

    def get_visibility(self):
        return self.visible

    def __hash__(self):
        return hash(id(self))

    def __eq__(self, other):
        return id(self) == id(other)

    def get_draw_batch(self):
        # Somewhat obscure code recursively compiles list of drawing instructions to be sent to window for drawing
        return [self.visible_children[id].get_draw_batch() for id in self.visible_children].append(self.draw_instructions)


class Frame(Interface):
    def __init__(self, x_i, y_i, x_f, y_f, screen_width, screen_height, color):
        super().__init__()
        self.color = color
        self.x_i, self.x_f = screen_width * x_i, screen_width * x_f
        self.y_i, self.y_f = screen_height * y_i, screen_height * y_f
        self.draw_instructions.append([4, gl.GL_QUADS, None,
                                       ('v2f', (x_i, y_i, x_f, y_i, x_f, y_f, x_i, y_f)), self.color])


class Textbox(Interface):
    def __init__(self, x_i, y_i, x_f, y_f, screen_width, screen_height, border_color, fill_color):
        super().__init()


class Button(Interface):
    def mouse_press(self, signal_type, **kwargs):
        print(signal_type, kwargs)


class Checkbox(Button):
    pass

class PushButton(Button):
    pass


class Plot(Interface):
    pass


class Display(Interface):
    pass
